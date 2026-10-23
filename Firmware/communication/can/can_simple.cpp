#include "can_simple.hpp"

#include <odrive_main.h>

#include <math.h>
#include <cmath>
#include <cstring>

// 版本信息读取 (定义见文件末尾, 数据由 .version_info 段提供)
uint16_t get_hardware_version(void);
uint32_t get_software_version(void);
uint32_t get_publish_data(void);

/* ------------------------------------------------------------------------- */
/* 本文件移植自 odrive-vlago 工程的 communication/can_simple.cpp             */
/* 接收侧改为 "2 个硬件 bank + 一个 switch (msg.id) 分发";                     */
/* 原来 ODrive 标准的 do_command() (nodeID|cmdID) 解析已停用。                 */
/* ------------------------------------------------------------------------- */

// 周期参数 (与参考工程一致)
#define VEL_ESTIMATE_PERIOD_MS (20)   // 0x21C 速度反馈
#define AXIS_ERROR_PERIOD_MS (1000)   // 0x270/0x271
#define MOTOR_ERROR_PERIOD_MS (1005)  // 0x272/0x273
#define ENCODER_ERROR_PERIOD_MS (1010)  // 0x275/0x276
#define ENCODER_COUNT_PERIOD_MS (50)  // 0x21A/0x21E
#define HEARTBEAT_PERIOD_MS (100)     // 0x274, 内部每 5 次真正发一帧 (500ms)

#define MAX_MSG_LOST_COUNT (3)  // 指令丢失判定 (100ms * 4 = 400ms)
#define BRAKE_FILTER_COUNT (6)  // 手动抱闸 IO 滤波次数

// 抱闸动作阈值 (单位: turn/s)
#define BREAK_SPEED_MT (2.0f)
#define MIN_ZERO_IDLE_CNT (12)  // 抱闸后再延时 12 个指令周期切 IDLE

#ifdef ENCODER_USE_SPI_ABS_AMS
#define MIN_ZERO_SPEED_CNT (750)  // 磁编: 速度小于阈值后 15s 抱闸
#elif defined(ENCODER_USE_HALL)
#define MIN_ZERO_SPEED_CNT (50)  // 霍尔
#else
#define MIN_ZERO_SPEED_CNT (750)
#endif

// PID 参数 (可通过 0x208 修改)
float PID_P = 0.1f;
float PID_I = 0.05f;
float PID_P_ENCODER_ABS = 0.15f;
float PID_I_ENCODER_ABS = 0.1f;

uint8_t g_identify_mode = 0;   // 进入识别模式
uint8_t g_level = 1;          // 摇杆档位 (预留)
uint8_t g_drive_mode = 1;     // 驾驶模式: 1-舒适 2-运动
uint8_t g_operate_mode = 4;   // 运行模式: 3-推行, 4-标准, 7-狭窄空间

uint32_t CANSimple::msg_lost_count_ = 0;
bool CANSimple::brake_sts_ = false;
bool CANSimple::back_push_sts_ = false;
bool CANSimple::brake_released_[AXIS_COUNT] = {false, false};

/* ========================================================================= */
/* 摇杆(ADC)差速控制 —— 移植自 odrive-vlago can_simple.cpp (生效版本:          */
/* "优化低速转弯顿挫+三挡转弯降速")                                            */
/* 输入: 0x279 的 ADC 值; 输出: 左右轮目标转速 (turn/s)                        */
/* 注意: JOY_ADC_* / 档位表是实车标定参数, 换摇杆或换车需要重新标定            */
/* ========================================================================= */

#define SPORT_MODE 1             // 1-运动模式 0-舒适模式

// 摇杆 ADC 标定 (白车摇杆)
#define WHEELBASE_M 0.62f            // 轮距 [m]
#define JOY_ADC_X_MIN (0x0175 + 50)  // X 轴(左右)最小值
#define JOY_ADC_X_MAX (0x0E05 - 50)  // X 轴(左右)最大值
#define JOY_ADC_X_CENTER 0x082E      // X 轴中位值
#define JOY_ADC_Y_MIN (0x01A1 + 50)  // Y 轴(前后)最小值
#define JOY_ADC_Y_MAX (0x0E22 - 50)  // Y 轴(前后)最大值
#define JOY_ADC_Y_CENTER 0x081B      // Y 轴中位值

// 控制参数
#define DEADZONE_X_AD 150            // 中位死区(ADC 值)
#define LOWPASS_ALPHA_X_SPORT 0.8f   // 运动模式 X 轴低通系数
#define LOWPASS_ALPHA_X_COMFORT 0.2f // 舒适模式 X 轴低通系数
#define RATE_ALPHA_X 1.0f            // X 轴变化率限制 (1 = 不限制)
#define DEADZONE_Y_AD 150            // 中位死区(ADC 值)
#define LOWPASS_ALPHA_Y 0.05f        // Y 轴低通系数
#define RATE_ALPHA_Y_SPORT 0.01f     // 运动模式 Y 轴变化率
#define RATE_ALPHA_Y_SPORT_GEAR1 0.03f  // 运动模式 1 档 Y 轴变化率
#define RATE_ALPHA_Y_COMFORT 0.005f  // 舒适模式 Y 轴变化率

// 档位参数 (1~4 档)
float MAX_MOTOR_RPS = 12.0f;         // 直行最大速度 [turn/s]
float MAX_MOTOR_RPS_REVERSE = 7.0f;  // 倒车最大速度
float MAX_MOTOR_RPS_SPIN_TURN = 5.0f;// 原地转圈最大速度
float TURN_SENSITIVITY = 0.8f;       // 转向灵敏度
float TURN_MARGIN = 3.0f;            // 转弯时外轮允许超出的限幅余量

static const float max_rps_table_comfort[] = {9.0f, 18.0f, 27.0f, 36.0f};
static const float max_rps_table_sport[] = {12.0f, 24.0f, 36.0f, 48.0f};
static const float max_rps_reverse_table[] = {7.0f, 9.0f, 11.0f, 13.0f};
static const float max_rps_spin_turn_table_comfort[] = {4.0f, 5.0f, 6.0f, 7.0f};
static const float max_rps_spin_turn_table_sport[] = {5.0f, 6.0f, 7.0f, 8.0f};
static const float turn_sensitivity_table[] = {0.8f, 0.55f, 0.45f, 0.4f};
static const float turn_margin_table[] = {20.0f, 25.0f, 35.0f, 50.0f};

#define MAX_LEVEL 4  // 档位数
static_assert((int)(sizeof(max_rps_table_sport) / sizeof(max_rps_table_sport[0])) == MAX_LEVEL,
              "档位参数表长度与 MAX_LEVEL 不一致");
#define GEAR_TRANSITION_ALPHA 0.03f  // 换档平滑系数
#define SPIN_WEIGHT_ALPHA 0.9f       // 原地转圈权重平滑系数

typedef struct {
    // 输入
    uint16_t adc_x_raw;  // 摇杆 X 轴原始 ADC 值
    uint16_t adc_y_raw;  // 摇杆 Y 轴原始 ADC 值

    // 内部状态
    float joy_x_filtered;  // 滤波后的 X 轴指令 (-1.0 ~ +1.0)
    float joy_y_filtered;  // 滤波后的 Y 轴指令 (-1.0 ~ +1.0)
    float left_speed_cmd;  // 计算出的左轮速度命令 [turn/s]
    float right_speed_cmd; // 计算出的右轮速度命令 [turn/s]

    // 换档平滑参数 (仅前进和后退)
    float max_motor_rps_smooth;
    float max_motor_rps_reverse_smooth;

    // 输出
    float left_speed_out;
    float right_speed_out;
} JoystickController_t;

static JoystickController_t joystick_ctrl;

static void JoystickCtrl_Init(JoystickController_t *ctrl) {
    if (ctrl == NULL) return;

    ctrl->adc_x_raw = JOY_ADC_X_CENTER;
    ctrl->adc_y_raw = JOY_ADC_Y_CENTER;
    ctrl->joy_x_filtered = 0.0f;
    ctrl->joy_y_filtered = 0.0f;
    ctrl->left_speed_cmd = 0.0f;
    ctrl->right_speed_cmd = 0.0f;
    ctrl->left_speed_out = 0.0f;
    ctrl->right_speed_out = 0.0f;
    ctrl->max_motor_rps_smooth = max_rps_table_sport[0];
    ctrl->max_motor_rps_reverse_smooth = max_rps_reverse_table[0];
}

// ADC 值映射到 [-1, 1], 中位死区内输出 0
static float _normalize_with_deadzone_optimized(uint16_t raw,
                                                uint16_t min_val,
                                                uint16_t max_val,
                                                uint16_t center_val,
                                                uint16_t deadzone_adc) {
    uint16_t deadzone_min = (center_val > deadzone_adc) ? center_val - deadzone_adc : min_val;
    uint16_t deadzone_max = (center_val + deadzone_adc < max_val) ? center_val + deadzone_adc : max_val;

    if (raw >= deadzone_min && raw <= deadzone_max) {
        return 0.0f;
    }

    if (raw < deadzone_min) {
        if (raw <= min_val) return -1.0f;
        uint16_t left_range = deadzone_min - min_val;
        if (left_range == 0) return -1.0f;
        return -1.0f * (float)(deadzone_min - raw) / (float)left_range;
    }

    if (raw >= max_val) return 1.0f;
    uint16_t right_range = max_val - deadzone_max;
    if (right_range == 0) return 1.0f;
    return (float)(raw - deadzone_max) / (float)right_range;
}

// 一阶低通滤波
static float _lowpass_filter(float new_value, float old_value, float alpha) {
    float value = alpha * new_value + (1.0f - alpha) * old_value;
    if (fabsf(value) < 0.001f) value = 0.0f;  // 小值置零, 避免漂移
    return value;
}

// 变化率限制, 防突变
static float _rate_limit(float new_cmd, float old_cmd, float max_accel_delta, float max_reduce_delta) {
    if (fabsf(new_cmd) < 0.001f) return 0.0f;

    float delta = new_cmd - old_cmd;
    float abs_new = fabsf(new_cmd);
    float abs_old = fabsf(old_cmd);
    float limit;

    if ((new_cmd > 0 && old_cmd < 0) || (new_cmd < 0 && old_cmd > 0)) {
        limit = max_reduce_delta;  // 方向改变, 采用更严格的限制
    } else if (abs_new > abs_old) {
        limit = max_accel_delta;
    } else {
        limit = max_reduce_delta;
    }

    if (fabsf(delta) > limit) {
        return (delta > 0) ? (old_cmd + limit) : (old_cmd - limit);
    }
    return new_cmd;
}

// 摇杆 ADC -> 左右轮目标转速
static void JoystickCtrl_Update(JoystickController_t *ctrl, uint16_t adc_x, uint16_t adc_y,
                                uint8_t gear_num, uint8_t drive_mode) {
    if (ctrl == NULL) return;
    ctrl->adc_x_raw = adc_x;
    ctrl->adc_y_raw = adc_y;

    // 标定到 [-1,1] 并应用死区
    float joy_x_normalized = _normalize_with_deadzone_optimized(adc_x, JOY_ADC_X_MIN, JOY_ADC_X_MAX,
                                                                JOY_ADC_X_CENTER, DEADZONE_X_AD);
    float joy_y_normalized = _normalize_with_deadzone_optimized(adc_y, JOY_ADC_Y_MIN, JOY_ADC_Y_MAX,
                                                                JOY_ADC_Y_CENTER, DEADZONE_Y_AD);
    joy_x_normalized = -1.0f * joy_x_normalized;  // X 轴方向修正

    // --- 指令滤波 ---
    static float last_joy_x = 0.0f, last_joy_y = 0.0f;
    if (drive_mode == 0) {  // 舒适模式
        ctrl->joy_x_filtered = _lowpass_filter(joy_x_normalized, ctrl->joy_x_filtered, LOWPASS_ALPHA_X_COMFORT);
        ctrl->joy_y_filtered = _lowpass_filter(joy_y_normalized, ctrl->joy_y_filtered, LOWPASS_ALPHA_Y);
        if (joy_y_normalized != 0) {
            ctrl->joy_y_filtered = _rate_limit(ctrl->joy_y_filtered, last_joy_y, RATE_ALPHA_Y_COMFORT, 1);
        }
    } else {  // 运动模式
        ctrl->joy_x_filtered = _lowpass_filter(joy_x_normalized, ctrl->joy_x_filtered, LOWPASS_ALPHA_X_SPORT);
        ctrl->joy_y_filtered = _lowpass_filter(joy_y_normalized, ctrl->joy_y_filtered, LOWPASS_ALPHA_Y);

        if (fabsf(joy_y_normalized) < 0.1f) {
            ctrl->joy_x_filtered = _rate_limit(ctrl->joy_x_filtered, last_joy_x, RATE_ALPHA_X, 0.03f);
        }
        if ((joy_y_normalized != 0) && (gear_num != 1)) {
            ctrl->joy_y_filtered = _rate_limit(ctrl->joy_y_filtered, last_joy_y, RATE_ALPHA_Y_SPORT, 1);
        } else if ((joy_y_normalized != 0) && (gear_num == 1)) {
            ctrl->joy_y_filtered = _rate_limit(ctrl->joy_y_filtered, last_joy_y, RATE_ALPHA_Y_SPORT_GEAR1, 1);
        }
    }
    last_joy_x = ctrl->joy_x_filtered;
    last_joy_y = ctrl->joy_y_filtered;

    // --- 档位参数 ---
    uint8_t gear_index = (gear_num > 0 && gear_num <= MAX_LEVEL) ? (uint8_t)(gear_num - 1) : 0;

    float target_max_motor_rps;
    float target_max_motor_rps_reverse = max_rps_reverse_table[gear_index];
    if (drive_mode == 0) {
        target_max_motor_rps = max_rps_table_comfort[gear_index];
    } else {
        target_max_motor_rps = max_rps_table_sport[gear_index];
    }

    // 只对前进/后退最高速做换档平滑
    ctrl->max_motor_rps_smooth = _lowpass_filter(target_max_motor_rps, ctrl->max_motor_rps_smooth, GEAR_TRANSITION_ALPHA);
    ctrl->max_motor_rps_reverse_smooth = _lowpass_filter(target_max_motor_rps_reverse, ctrl->max_motor_rps_reverse_smooth, GEAR_TRANSITION_ALPHA);

    MAX_MOTOR_RPS = ctrl->max_motor_rps_smooth;
    MAX_MOTOR_RPS_REVERSE = ctrl->max_motor_rps_reverse_smooth;

    TURN_SENSITIVITY = turn_sensitivity_table[gear_index];
    TURN_MARGIN = turn_margin_table[gear_index];
    if (drive_mode == 0) {
        MAX_MOTOR_RPS_SPIN_TURN = max_rps_spin_turn_table_comfort[gear_index];
    } else {
        MAX_MOTOR_RPS_SPIN_TURN = max_rps_spin_turn_table_sport[gear_index];
    }

    // --- 原地转圈模式 ---
    float spin_speed = ctrl->joy_x_filtered * MAX_MOTOR_RPS_SPIN_TURN;
    float spin_left = -spin_speed;  // 左轮反向
    float spin_right = spin_speed;  // 右轮正向

    // --- 转向半径模型 ---
    float joy_x_abs = fabsf(ctrl->joy_x_filtered);
    float joy_y_abs = fabsf(ctrl->joy_y_filtered);
    float base_speed = (ctrl->joy_y_filtered >= 0.0f) ?
                       (ctrl->joy_y_filtered * MAX_MOTOR_RPS) :
                       (ctrl->joy_y_filtered * MAX_MOTOR_RPS_REVERSE);
    float turn_gain = ctrl->joy_x_filtered * TURN_SENSITIVITY;
    if (ctrl->joy_y_filtered < 0.0f) {
        turn_gain = -turn_gain;  // 倒车时转向反向
    }
    float radius_left = base_speed * (1.0f - turn_gain);
    float radius_right = base_speed * (1.0f + turn_gain);

    // 动态限幅: 外轮饱和时整体缩放, 保持转向
    float max_allowed = max_rps_table_sport[MAX_LEVEL - 1];
    float abs_max = fabsf(max_allowed);
    if (fabsf(radius_left) > abs_max + 0.1f || fabsf(radius_right) > abs_max + 0.1f) {
        float max_speed = fmaxf(fabsf(radius_left), fabsf(radius_right));
        float scale = abs_max / max_speed;
        radius_left *= scale;
        radius_right *= scale;
    }

    // --- 两种模型平滑切换 ---
    float spin_weight_raw = (1.0f - joy_y_abs) * joy_x_abs;
    if (spin_weight_raw < 0.0f) spin_weight_raw = 0.0f;
    if (spin_weight_raw > 1.0f) spin_weight_raw = 1.0f;

    static float spin_weight_filtered = 0.0f;
    if (joy_y_abs >= 0.2f) {
        float alpha = 0.1f * joy_y_abs * SPIN_WEIGHT_ALPHA;  // y 较大: 慢平滑
        spin_weight_filtered = _lowpass_filter(spin_weight_raw, spin_weight_filtered, alpha);
    } else if (joy_y_abs >= 0.001f) {
        float alpha = 0.5f * (1.0f - joy_y_abs / 0.2f);  // y 较小: 快平滑
        spin_weight_filtered = _lowpass_filter(spin_weight_raw, spin_weight_filtered, alpha);
    } else {
        spin_weight_filtered = spin_weight_raw;  // y=0: 直接使用
    }

    float spin_weight = spin_weight_filtered;
    float radius_weight = 1.0f - spin_weight;

    ctrl->left_speed_cmd = radius_left * radius_weight + spin_left * spin_weight;
    ctrl->right_speed_cmd = radius_right * radius_weight + spin_right * spin_weight;

    ctrl->left_speed_out = ctrl->left_speed_cmd;
    ctrl->right_speed_out = ctrl->right_speed_cmd;
}

bool CANSimple::init() {
    // bank0: 扩展帧 0x200~0x2FF, 一条掩码覆盖
    //        0x208 / 0x217 / 0x219 / 0x21B / 0x21D / 0x277
    //        以及上控/摇杆相关: 0x220 / 0x221 / 0x222 / 0x229 / 0x279
    // 掩码 0x700 只比较 bit8~bit10, 因此只有 0x2xx 的扩展帧能进来;
    // 具体是哪条命令由 handle_can_message() 的 switch (msg.id) 精确分发。
    MsgIdFilterSpecs cmd_filter = {
        .id = (uint32_t)0x200,
        .mask = (uint32_t)0x700};
    if (!canbus_->subscribe(cmd_filter, &CANSimple::can_message_callback, this, &subscription_handle_)) {
        return false;
    }

    // bank1: 扩展帧 0x701 精确匹配 (OTA/复位, 与 bootloader 一致, 29 位全掩码)
    MsgIdFilterSpecs ota_filter = {
        .id = (uint32_t)MSG_OTA_RESET,
        .mask = (uint32_t)0x1FFFFFFF};
    return canbus_->subscribe(ota_filter, &CANSimple::can_message_callback, this, &ota_subscription_handle_);
}

void CANSimple::can_message_callback(void* ctx, const can_Message_t& msg) {
    static_cast<CANSimple*>(ctx)->handle_can_message(msg);
}

// 前向声明: 定义在下方 "磁编 AS5047P 零位标定" 一节, 这里分发时要用
static bool as5047_has_cmd_tail(const can_Message_t& msg);

void CANSimple::handle_can_message(const can_Message_t& msg) {
    if (!msg.isExt) {
        return;
    }

    // 识别/校准过程中只接受 0x277 本身 (OTA 复位始终允许)
    if ((g_identify_mode != 0) && (msg.id != MSG_IDENTIFY) && (msg.id != MSG_OTA_RESET)) {
        return;
    }

    // 单一入口, 按固定 CAN ID 分发 (do_command 已停用)
    switch (msg.id) {
        case MSG_SET_AXIS_CONFIG_PID:  // 0x208 设置 PID
            set_axis_pid_config_callback(msg);
            break;

        case MSG_GET_VBUS_VOLTAGE:  // 0x217 获取母线电压/电流
            get_vbus_voltage_callback(msg);
            break;

        //case 0x290:        
        case MSG_SET_VLAGO_INPUT_VEL:  // 0x219 域控速度指令
        //case MSG_SET_UP_CONTROLL_VEL:  // 0x220 上控速度指令
        //case MSG_SET_VLAGO_ADC:        // 0x279 摇杆 ADC (本地摇杆控车)
            set_vlago_input_vel_callback(msg);
            break;

        case MSG_SET_VLAGO_BRAKE_VEL:  // 0x21B 抱闸控制
            set_vlago_brake_vel_callback(msg);
            break;

        case MSG_SET_OPERATE_MODE:  // 0x221 运行模式
            set_operate_mode_callback(msg);
            break;

        case MSG_SET_LEVEL:  // 0x222 档位
            set_level_callback(msg);
            break;

        case MSG_SET_DRIVE_MODE:  // 0x229 驾驶模式
            set_drive_mode_callback(msg);
            break;

        case MSG_GET_VLAGO_VERSION:  // 0x21D 获取版本信息
            get_vlago_version_callback(msg);
            break;

        case MSG_IDENTIFY: {  // 0x277: 磁编零位标定 (尾缀 01..07) 或 识别/校准 (尾缀 11..77)
            // 两者共用 0x277, 靠 byte1~byte7 的固定尾缀区分, 互不重叠
            if (as5047_has_cmd_tail(msg)) {
                const uint8_t sub = msg.buf[0];
                if (sub == 0x00) {
                    as5047_read_diag_callback(msg);      // 读磁场强度/诊断
                } else if (sub == 0x33) {
                    as5047_zero_offset_callback(msg);    // 软写 ZPOS (易失)
                } else if (sub == 0x44) {
                    as5047_burn_otp_callback(msg);       // 烧 OTP (不可逆)
                } else {
                    phase_lock_zero_callback(msg);       // 0x11 锁定 / 0x22 退出
                }
            } else {
                identify_callback(msg);
            }
        } break;

        case MSG_OTA_RESET:  // 0x701 OTA 复位
            handle_ota_message(msg);
            break;

        default:
            break;
    }

    // NOTE: 原来的 per-axis 匹配 + do_command() 已停用, 说明见文件末尾注释
}

/* ------------------------------------------------------------------------- */
/* 接收命令处理                                                               */
/* ------------------------------------------------------------------------- */

// 0x208: 设置 PID, 并把实际生效的编码器 PID 回读给上位机
void CANSimple::set_axis_pid_config_callback(const can_Message_t& msg) {
    PID_P = can_getSignal<float>(msg, 0, 32, true);
    PID_I = can_getSignal<float>(msg, 32, 32, true);
    PID_P_ENCODER_ABS = PID_P;
    PID_I_ENCODER_ABS = PID_I;

    can_Message_t txmsg;
    txmsg.id = MSG_SET_AXIS_CONFIG_PID;
    txmsg.isExt = true;
    txmsg.len = 8;
    can_setSignal<float>(txmsg, PID_P_ENCODER_ABS, 0, 32, true);
    can_setSignal<float>(txmsg, PID_I_ENCODER_ABS, 32, 32, true);
    canbus_->send_message(txmsg);
}

// 0x217: 母线电压/电流回读
void CANSimple::get_vbus_voltage_callback(const can_Message_t& msg) {
    if (!(msg.rtr || msg.len == 0)) {
        return;
    }

    can_Message_t txmsg;
    txmsg.id = MSG_GET_VBUS_VOLTAGE;
    txmsg.isExt = true;
    txmsg.len = 8;
    can_setSignal<float>(txmsg, vbus_voltage, 0, 32, true);
    can_setSignal<float>(txmsg, ibus_, 32, 32, true);
    canbus_->send_message(txmsg);
}

// 0x219: 速度指令 (左右轮各一个 float), 含抱闸/状态机/增益调度逻辑
void CANSimple::set_vlago_input_vel_callback(const can_Message_t& msg) {
    msg_lost_count_ = 0;

    // 推行/手动抱闸/错误状态 -> 直接停机
    if (back_push_sts_ || brake_sts_) {
        stop_axes();
        return;
    }
    for (auto& axis : axes) {
        if (axis.error_ != Axis::ERROR_NONE) {
            stop_axes();
            return;
        }
    }

    float input_vel[AXIS_COUNT] = {0.0f, 0.0f};

    // 摇杆控制器只需初始化一次 (必须在第一次 JoystickCtrl_Update 之前)
    static bool joystick_inited = false;
    if (!joystick_inited) {
        joystick_inited = true;
        JoystickCtrl_Init(&joystick_ctrl);
    }

    if (msg.id == MSG_SET_VLAGO_ADC) {  // 0x279 摇杆 AD 值 -> 差速控制
        if (msg.len < 6) {              // 必须带 X/Y 两个 16bit 值, 否则丢弃
            return;
        }
        const uint16_t adc_x = (uint16_t)(msg.buf[4] | (msg.buf[5] << 8));
        const uint16_t adc_y = (uint16_t)(msg.buf[0] | (msg.buf[1] << 8));
#if SPORT_MODE
        JoystickCtrl_Update(&joystick_ctrl, adc_x, adc_y, g_level, 1);  // 运动模式
#else
        JoystickCtrl_Update(&joystick_ctrl, adc_x, adc_y, g_level, 0);  // 舒适模式
#endif
#ifdef ENCODER_USE_SPI_ABS_AMS
        input_vel[0] = -1.0f * joystick_ctrl.left_speed_out;  // 磁编: 轴0 方向取反
#else
        input_vel[0] = joystick_ctrl.left_speed_out;
#endif
        input_vel[1] = joystick_ctrl.right_speed_out;
    } else {  // 0x219 / 0x220 上控速度值 (两个 float)
        if (msg.len < 8) {  // 帧长不足, 丢弃
            return;
        }
#ifdef ENCODER_USE_SPI_ABS_AMS
        // 磁编: 轴0 方向取反 (与参考工程一致)
        input_vel[0] = -1.0f * can_getSignal<float>(msg, 0, 32, true);
#else
        input_vel[0] = can_getSignal<float>(msg, 0, 32, true);
#endif
        input_vel[1] = can_getSignal<float>(msg, 32, 32, true);
    }

    // 上电仅设置一次控制模式 (速度控制 + 速度斜坡)
    static bool mode_configured = false;
    if (!mode_configured) {
        mode_configured = true;
        for (auto& axis : axes) {
            if (axis.controller_.config_.control_mode != Controller::CONTROL_MODE_VELOCITY_CONTROL) {
                axis.controller_.config_.control_mode = Controller::CONTROL_MODE_VELOCITY_CONTROL;
                axis.controller_.control_mode_updated();
            }
            if (axis.controller_.config_.input_mode != Controller::INPUT_MODE_VEL_RAMP) {
                axis.controller_.config_.input_mode = Controller::INPUT_MODE_VEL_RAMP;
            }
        }
    }

    static uint16_t axes_wait[AXIS_COUNT] = {};
    static uint16_t axes_zero[AXIS_COUNT] = {};
    static uint16_t idle_status[AXIS_COUNT] = {};
    static uint16_t idle_cnt[AXIS_COUNT] = {};

    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        Axis& axis = axes[i];
        const float vel_est = axis.encoder_.vel_estimate_.any().value_or(0.0f);

        // 1---速度给定为 0 且速度反馈足够小: 延时抱闸, 再延时切 IDLE
        if ((input_vel[i] == 0.0f) && (std::fabs(vel_est) <= BREAK_SPEED_MT)) {
            if (brake_released_[i]) {
                axes_zero[i]++;
                if (axes_zero[i] >= MIN_ZERO_SPEED_CNT) {
                    axes_zero[i] = 0;
                    if (axis.encoder_.config_.pre_calibrated) {
                        set_brake(i, false);  // 抱闸
                        idle_status[i] = 1;
                    }
                }
            } else {
                axes_zero[i] = 0;
            }
        }

        // 抱闸后延时 120ms 再切 IDLE (参考工程把这段嵌套在上面的 if 里,
        // 抱闸后条件不再成立, 实际永远切不到 IDLE, 这里按注释意图修正)
        if (idle_status[i]) {
            idle_cnt[i]++;
            if (idle_cnt[i] >= MIN_ZERO_IDLE_CNT) {
                axis.requested_state_ = Axis::AXIS_STATE_IDLE;
                idle_cnt[i] = 0;
                idle_status[i] = 0;
            }
        }

        // 2---速度给定不为 0: 开闸, 延时 120ms 再给速度, 避免冲击机械抱闸
        if (input_vel[i] != 0.0f) {
            idle_status[i] = 0;
            idle_cnt[i] = 0;
            if (axis.encoder_.config_.pre_calibrated) {
                set_brake(i, true);  // 开闸
                if (axis.current_state_ != Axis::AXIS_STATE_CLOSED_LOOP_CONTROL) {
                    axis.requested_state_ = Axis::AXIS_STATE_CLOSED_LOOP_CONTROL;
                }
            }
            if (axes_wait[i] < MIN_ZERO_IDLE_CNT / 2) {
                axes_wait[i]++;
                input_vel[i] = 0.0f;
            }
        } else {
            axes_wait[i] = 0;
        }

        axis.controller_.input_vel_ = input_vel[i];
        axis.watchdog_feed();  // 收到有效速度指令即喂狗

#ifdef ENCODER_USE_HALL
        // 霍尔带宽随转速切换
        axis.encoder_.config_.bandwidth = (std::fabs(vel_est) < 5.0f) ? 20.0f : 60.0f;
#endif

        // 3---驻坡: 零速附近且电流较大时提高增益, 稳定后恢复
        // 编译开关见 board.h: 注释掉 ENABLE_ZHUPO_GAIN_SCHED 即整体关闭该调度,
        // 关闭后增益始终跟随上位机 (0x208 / 0x221) 下发的基准 PID, 便于 A/B 对比。
#ifdef ENABLE_ZHUPO_GAIN_SCHED
        static uint8_t zhupo_cnt[AXIS_COUNT] = {};
        static uint8_t zhupo_flag[AXIS_COUNT] = {};
        if ((std::fabs(input_vel[i]) < 1.0f) && (std::fabs(vel_est) <= 5.0f)) {
            if (zhupo_flag[i] == 0) {
                zhupo_cnt[i]++;
                if (zhupo_cnt[i] >= 250) {
                    zhupo_flag[i] = 1;
                    zhupo_cnt[i] = 0;
#ifdef ENCODER_USE_SPI_ABS_AMS
                    axis.controller_.config_.vel_gain = PID_P_ENCODER_ABS;
                    axis.controller_.config_.vel_integrator_gain = PID_I_ENCODER_ABS;
#else
                    axis.controller_.config_.vel_gain = PID_P;
                    axis.controller_.config_.vel_integrator_gain = PID_I;
#endif
                } else {
                    if (std::fabs(axis.motor_.current_control_.Iq_measured_) > 3.0f) {
#ifdef ENCODER_USE_SPI_ABS_AMS
                        axis.controller_.config_.vel_gain = 0.3f;
                        axis.controller_.config_.vel_integrator_gain = 0.4f;
#else
                        axis.controller_.config_.vel_gain = 0.2f;
                        axis.controller_.config_.vel_integrator_gain = 0.3f;
#endif
                    }
                }
            }
        } else {
            zhupo_flag[i] = 0;
            zhupo_cnt[i] = 0;
#ifdef ENCODER_USE_SPI_ABS_AMS
            axis.controller_.config_.vel_gain = PID_P_ENCODER_ABS;
            axis.controller_.config_.vel_integrator_gain = PID_I_ENCODER_ABS;
#else
            axis.controller_.config_.vel_gain = PID_P;
            axis.controller_.config_.vel_integrator_gain = PID_I;
#endif
        }
#else   // !ENABLE_ZHUPO_GAIN_SCHED: 不做驻坡增益调度, 始终用上位机下发值
#ifdef ENCODER_USE_SPI_ABS_AMS
        axis.controller_.config_.vel_gain = PID_P_ENCODER_ABS;
        axis.controller_.config_.vel_integrator_gain = PID_I_ENCODER_ABS;
#else
        axis.controller_.config_.vel_gain = PID_P;
        axis.controller_.config_.vel_integrator_gain = PID_I;
#endif
#endif  // ENABLE_ZHUPO_GAIN_SCHED
    }
}

// 0x21B: 抱闸控制 (两位, 1-解除抱闸 0-抱闸), 并回读抱闸状态
void CANSimple::set_vlago_brake_vel_callback(const can_Message_t& msg) {
    const int16_t brake_0_sts = can_getSignal<int16_t>(msg, 0, 16, true);
    const int16_t brake_1_sts = can_getSignal<int16_t>(msg, 16, 16, true);

    set_brake(0, brake_0_sts != 0);
    set_brake(1, brake_1_sts != 0);

    if (brake_0_sts && brake_1_sts) {
        // 两轴都被要求抱闸 -> 按推行处理, 速度指令作废
        back_push_sts_ = true;
        for (auto& axis : axes) {
            axis.requested_state_ = Axis::AXIS_STATE_IDLE;
        }
    } else if ((0 == brake_0_sts) && (0 == brake_1_sts)) {
        back_push_sts_ = false;
    }

    can_Message_t txmsg;
    txmsg.id = MSG_SET_VLAGO_BRAKE_VEL;
    txmsg.isExt = true;
    txmsg.len = 8;
    txmsg.buf[0] = brake_released_[0] ? 1 : 0;
    txmsg.buf[1] = 0;
    txmsg.buf[2] = brake_released_[1] ? 1 : 0;
    txmsg.buf[3] = 0;
    txmsg.buf[4] = 1;
    txmsg.buf[5] = 2;
    txmsg.buf[6] = 3;
    txmsg.buf[7] = 4;
    canbus_->send_message(txmsg);
}

// 0x221: 运行模式, 模式 7 (狭窄空间) 时切换到另一组 PID
void CANSimple::set_operate_mode_callback(const can_Message_t& msg) {
    if (msg.len < 2) {
        return;
    }
    g_operate_mode = msg.buf[1];
    if (g_operate_mode == 7) {
        PID_P = 0.3f;
        PID_I = 0.5f;
        PID_P_ENCODER_ABS = 0.3f;
        PID_I_ENCODER_ABS = 0.5f;
    } else {
        PID_P = 0.1f;
        PID_I = 0.05f;
        PID_P_ENCODER_ABS = 0.15f;
        PID_I_ENCODER_ABS = 0.1f;
    }
}

// 0x222: 档位 (1~4)
void CANSimple::set_level_callback(const can_Message_t& msg) {
    if (msg.len < 1) {
        return;
    }
    if ((msg.buf[0] >= 1) && (msg.buf[0] <= MAX_LEVEL)) {
        g_level = msg.buf[0];
    }
}

// 0x229: 驾驶模式 1-舒适 2-运动
void CANSimple::set_drive_mode_callback(const can_Message_t& msg) {
    if (msg.len < 1) {
        return;
    }
    if ((msg.buf[0] == 0x01) || (msg.buf[0] == 0x02)) {
        g_drive_mode = msg.buf[0];
    }
}

// 0x21D: 版本信息 (硬件版本 / 软件版本 / 发布日期)
void CANSimple::get_vlago_version_callback(const can_Message_t& msg) {
    (void)msg;
    const uint16_t hardware_version = get_hardware_version();
    const uint16_t software_version = (uint16_t)get_software_version();
    const uint32_t publish_data = get_publish_data();

    can_Message_t txmsg;
    txmsg.id = MSG_GET_VLAGO_VERSION;
    txmsg.isExt = true;
    txmsg.len = 8;
    txmsg.buf[0] = hardware_version >> 8;
    txmsg.buf[1] = hardware_version & 0xFF;
    txmsg.buf[2] = software_version >> 8;
    txmsg.buf[3] = software_version & 0xFF;
    txmsg.buf[4] = publish_data >> 24;
    txmsg.buf[5] = publish_data >> 16;
    txmsg.buf[6] = publish_data >> 8;
    txmsg.buf[7] = publish_data & 0xFF;
    canbus_->send_message(txmsg);
}

/* ------------------------------------------------------------------------- */
/* 磁编 AS5047P 零位标定: CAN 0x277 (命令) / 0x278 (应答)                        */
/* byte1~byte7 = 01 02 03 04 05 06 07 固定校验字, 与识别命令 (11..77) 区分       */
/* ------------------------------------------------------------------------- */

// 校验 byte1~byte7 固定尾缀 01 02 03 04 05 06 07
static bool as5047_has_cmd_tail(const can_Message_t& msg) {
    static const uint8_t expected[7] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
    if (msg.len < 8) {
        return false;
    }
    for (size_t i = 0; i < sizeof(expected); ++i) {
        if (msg.buf[i + 1] != expected[i]) {
            return false;
        }
    }
    return true;
}

// 0x278 应答: byte0 = 命令回显, byte1 = 数据高 8 位, byte2 = 数据低 8 位,
//            byte3 = 结果码 (0/未用 = 成功, 非 0 = 失败原因), byte4-7 = 0
void CANSimple::send_count_reply(uint8_t cmd_echo, uint16_t count, uint8_t code) {
    can_Message_t txmsg;
    txmsg.id = PHASE_LOCK_REPLY_MSG_ID;
    txmsg.isExt = true;
    txmsg.len = 8;
    can_setSignal<uint8_t>(txmsg, cmd_echo, 0, 8, true);
    can_setSignal<uint8_t>(txmsg, (uint8_t)(count >> 8), 8, 8, true);     // byte1 = 高 8 位
    can_setSignal<uint8_t>(txmsg, (uint8_t)(count & 0xffu), 16, 8, true); // byte2 = 低 8 位
    can_setSignal<uint8_t>(txmsg, code, 24, 8, true);                     // byte3 = 结果码
    // byte4-7 已经是 0

    canbus_->send_message(txmsg);
}

// 0x278 诊断应答: byte0 = 命令回显(0x00), byte1/2 = 磁场强度, byte3 = 诊断状态位,
//                byte4 = AGC 值, byte5-7 = 0
void CANSimple::send_diag_reply(uint8_t cmd_echo, uint16_t magnitude, uint8_t status, uint8_t agc) {
    can_Message_t txmsg;
    txmsg.id = PHASE_LOCK_REPLY_MSG_ID;
    txmsg.isExt = true;
    txmsg.len = 8;
    can_setSignal<uint8_t>(txmsg, cmd_echo, 0, 8, true);
    can_setSignal<uint8_t>(txmsg, (uint8_t)(magnitude >> 8), 8, 8, true);     // byte1 = 高 8 位
    can_setSignal<uint8_t>(txmsg, (uint8_t)(magnitude & 0xffu), 16, 8, true); // byte2 = 低 8 位
    can_setSignal<uint8_t>(txmsg, status, 24, 8, true);                       // byte3 = 状态位
    can_setSignal<uint8_t>(txmsg, agc, 32, 8, true);                          // byte4 = AGC
    // byte5-7 已经是 0

    canbus_->send_message(txmsg);
}

// 0x00: 读 AS5047 的磁场强度和诊断信息 (仅 axis0)
// 流程: 读 MAG(0x3FFD) → 读 DIAAGC(0x3FFC), 纯读操作、无副作用
// 应答 byte3 状态位: bit0 = LF 偏移补偿完成, bit1 = COF CORDIC 溢出,
//                    bit2 = MAGH 磁场过强, bit3 = MAGL 磁场过弱
void CANSimple::as5047_read_diag_callback(const can_Message_t& msg) {
    if (!as5047_has_cmd_tail(msg)) {
        return;
    }

    const uint16_t INVALID = 0xffffu;
    uint16_t magnitude = INVALID;
    uint16_t diaagc = INVALID;

    axes[0].watchdog_feed();
    const bool ok = axes[0].encoder_.as5047_read_diagnostics(&magnitude, &diaagc, 50);
    axes[0].watchdog_feed();

    if (!ok) {
        send_diag_reply(0x00, INVALID, 0xffu, 0xffu);
        return;
    }

    // DIAAGC 位定义 (手册 Figure 21): bit8=LF, bit9=COF, bit10=MAGH, bit11=MAGL, bit7:0=AGC
    const uint8_t status = (uint8_t)((((diaagc >> 8) & 0x1u) << 0)    // bit0 = LF
                                   | (((diaagc >> 9) & 0x1u) << 1)    // bit1 = COF
                                   | (((diaagc >> 10) & 0x1u) << 2)   // bit2 = MAGH
                                   | (((diaagc >> 11) & 0x1u) << 3)); // bit3 = MAGL

    send_diag_reply(0x00, magnitude, status, (uint8_t)(diaagc & 0xffu));
}

// 0x11 期间读到的"电气 0 处的真实原始计数", 供 0x33 / 0x44 使用。
// 必须是 0x11 当时的值: 写 ZPOS 是**整体平移刻度**, 只要 X 是在电气 0 位置测到的,
// 之后无论转子停在哪里, 计数都会恰好在电气 0 处归零。若让 0x33 现场重读,
// 转子一旦漂移开 (0x22 解锁之后是常事) 就会把错误的零位固化下来。
static uint16_t as5047_cached_zero = 0xffffu;

// 上电后是否已经往 ZPOSM/ZPOSL 软写过东西。
// OTP 烧写前必须为 false —— 只有"没被软写过"时, 读回的 ZPOS 才等于 OTP 里的内容,
// 才能用"ZPOS 是否为 0"判断这颗芯片的 OTP 是否已经烧过。
static bool as5047_volatile_touched = false;

// OTP 序列是否正在进行 (结果由 service_stack() 每轮循环上报, 故意不阻塞 CAN 线程)
static bool as5047_otp_pending = false;

// 等 AXIS_STATE_PHASE_LOCK_ZERO 真正跑起来并且转子对齐到位
static bool wait_for_phase_lock_zero(Axis& axis) {
    // 1) 等状态机进入该状态
    bool entered = false;
    for (uint32_t i = 0; i < 500; ++i) {
        axis.watchdog_feed();
        if (axis.current_state_ == Axis::AXIS_STATE_PHASE_LOCK_ZERO) {
            entered = true;
            break;
        }
        osDelay(1);
    }
    if (!entered) {
        return false;
    }

    // 2) 电流按 phase_lock_ramp_time 爬到设定值, 再留一段固定稳定时间给转子对齐
    uint32_t settle_ms = (uint32_t)(axis.config_.phase_lock_ramp_time * 1000.0f) + 200u;
    for (uint32_t i = 0; i < settle_ms; ++i) {
        axis.watchdog_feed();
        osDelay(1);
    }
    return true;
}

// 电角度 0 锁定 (0x11) / 退出锁定 (0x22)
//   byte0 = 0x11 : 锁定到电角度 0 → 读计数 X → 设好软件侧对齐参数, 应答 0x278
//   byte0 = 0x22 : 退出锁定 (回到 IDLE, disarm), 无应答
void CANSimple::phase_lock_zero_callback(const can_Message_t& msg) {
    if (!as5047_has_cmd_tail(msg)) {
        return; // 校验字不符, 忽略整帧
    }

    const uint8_t cmd = can_getSignal<uint8_t>(msg, 0, 8, true);

    // ---- 0x22: 退出锁定 ----
    // 把 requested_state_ 从 UNDEFINED 改掉, run_phase_lock_zero() 的等待循环就会退出
    if (cmd == 0x22) {
        axes[0].requested_state_ = Axis::AXIS_STATE_IDLE;
        axes[0].watchdog_feed();
        return; // 无应答
    }

    if (cmd != 0x11) {
        return;
    }

    // ---- 0x11: 锁定到电角度 0 → 读计数 X → 设好软件侧对齐参数 ----
    // run_phase_lock_zero() 是开环直流对齐: 定子磁场固定在定子空间的电角度 0
    // (phase 源切到 open_loop_controller_.phase_, 不经过编码器),
    // 所以转子确实被拉到真实电角度 0, 与当前 phase_offset 对不对无关。
    axes[0].watchdog_feed();
    axes[0].requested_state_ = Axis::AXIS_STATE_PHASE_LOCK_ZERO;

    uint16_t x = 0xffffu;

    if (wait_for_phase_lock_zero(axes[0]) &&
        axes[0].encoder_.as5047_wait_still(500, 4)) {

        uint16_t zpos_now = 0xffffu;
        uint16_t count = 0xffffu;

        // 这里刻意**不写** ZPOS: 一是无副作用, 反复执行不会像早先那样把偏移一轮轮叠加;
        // 二是保证上电后读到的 ZPOS 始终等于 OTP 里的内容, 0x44 才能判断是否已烧过
        if (axes[0].encoder_.as5047_read_zero_and_count(&zpos_now, &count)) {
            // 硬件零位是 zpos_now, 所以 ANGLECOM = (原始角 - zpos_now) mod 16384,
            // 反解出真实原始角 X —— 这才是要写进 ZPOS 的值
            x = (uint16_t)((count + zpos_now) & 0x3fffu);
            as5047_cached_zero = x;

            // 当前硬件零位是 zpos_now, 电气 0 处的计数就是 count, 所以软件侧等效偏移
            // (phase_offset + phase_offset_float - 0.5) 必须等于 count
            axes[0].encoder_.config_.phase_offset = (int32_t)count;
            axes[0].encoder_.config_.phase_offset_float = 0.5f;
            // 这里必须直接赋值而不是用 setter: setter 会触发 check_pre_calibrated(),
            // 而 is_ready_ 恰恰要等 pre_calibrated 为 true 之后才由 abs_spi_cb 置起,
            // 走 setter 会互相等待、永远置不上 (死锁)
            axes[0].encoder_.config_.pre_calibrated = true;
            axes[0].watchdog_feed();
        }
    }

    send_count_reply(0x11, x);
}

// 0x33: 把 0x11 读到的 X 写入 ZPOSM/ZPOSL (仅 axis0)
// 应答 0x278: byte0 = 0x33, byte1/2 = 写入后读回的计数 (应 ≈ 0)
//             byte1 = byte2 = 0xFF 表示失败: 没先发 0x11 / 轴未静止 / ZPOS 写入校验没过
//
// 注意这里用的是 0x11 缓存的 X 而不是现场重读: 写 ZPOS 是整体平移刻度, 只要 X 是
// 在电气 0 位置测到的, 之后无论转子停在哪里, 计数都会恰好在电气 0 处归零。
void CANSimple::as5047_zero_offset_callback(const can_Message_t& msg) {
    if (!as5047_has_cmd_tail(msg)) {
        return;
    }

    if (as5047_cached_zero == 0xffffu) {
        // 没先发过 0x11, 没有可用的 X
        send_count_reply(0x33, 0xffffu);
        return;
    }

    const uint16_t x = as5047_cached_zero;
    axes[0].watchdog_feed();

    // 轴必须静止, 否则写 ZPOS 前后读数会漂
    if (!axes[0].encoder_.as5047_wait_still(500, 4)) {
        send_count_reply(0x33, 0xffffu);
        return;
    }

    uint16_t count = 0xffffu;
    uint16_t zposm_rb = 0xffffu;
    if (!axes[0].encoder_.as5047_write_zpos(x, &count, &zposm_rb) ||
        zposm_rb != (uint16_t)((x >> 6) & 0x00ffu)) {
        // ZPOSM 读回值和写入值不符, 说明没写进去, 此时不能动 phase_offset
        send_count_reply(0x33, 0xffffu);
        return;
    }

    // 硬件零位现在是 x, 即计数在电气 0 处归零, 故软件侧等效偏移必须为 0
    // (phase_offset + phase_offset_float - 0.5 = 0)
    axes[0].encoder_.config_.phase_offset = 0;
    axes[0].encoder_.config_.phase_offset_float = 0.5f;
    axes[0].encoder_.config_.pre_calibrated = true;
    as5047_volatile_touched = true; // ZPOS 被软写过了, 之后再想烧 OTP 必须先重新上电
    axes[0].watchdog_feed();

    send_count_reply(0x33, count);
}

// 0x44: 把 0x11 读到的 X 烧进 OTP (**不可逆, 只能烧一次**)
// 应答分两条 (ID 都是 0x278, DLC = 8): byte0 = 0x44, byte1/2 = 相关值, byte3 = 结果码
//   第 1 条 (立即): byte1/2 = 0xFFFF, byte3 = 0 表示"已开始", 非 0 为拒绝启动的原因码
//   第 2 条 (序列跑完后由 service_stack 主动上报): byte3 = 最终结果码, byte1/2 = 从 OTP 重建的零位
//   结果码: 0=成功(仅第1条) 1=SPI 超时 2=烧前比对不符 3=PROG 轮询超时 4=烧后比对不符
//           5=OTP 已烧过 6=前置条件不满足 7=上一次烧写还在进行中
//
// ★ 关键: 这里**绝不阻塞等待**。整条序列要跑几十到几百毫秒, 把 CAN 线程占住会让
//   HAL_CAN 的错误码被置起来, 而 can_server_thread 只处理 NONE/TIMEOUT, 其余会掉进
//   死循环 —— 那正是 CAN 彻底卡死的根源 (已在 odrive_can.cpp 里修掉)。
void CANSimple::as5047_burn_otp_callback(const can_Message_t& msg) {
    if (!as5047_has_cmd_tail(msg)) {
        return;
    }

    if (as5047_otp_pending) {
        send_count_reply(0x44, 0xffffu, 7); // 上一次还没跑完
        return;
    }

    // 前置条件: 先发过 0x11 拿到 X; 上电后没往 ZPOS 软写过 (否则无法预检是否已烧过);
    // 且不在闭环/锁定里 —— OTP 序列期间位置输出会失效, 带载运行会失控
    if (as5047_cached_zero == 0xffffu || as5047_volatile_touched ||
        axes[0].current_state_ != Axis::AXIS_STATE_IDLE) {
        send_count_reply(0x44, 0xffffu, 6);
        return;
    }

    axes[0].watchdog_feed();
    if (!axes[0].encoder_.as5047_burn_otp_start(as5047_cached_zero, 3000)) {
        send_count_reply(0x44, 0xffffu, 1);
        return;
    }
    as5047_otp_pending = true;

    send_count_reply(0x44, 0xffffu, 0); // 立即确认"已开始", 最终结果稍后单独上报
}

// 由 service_stack() 每轮 CAN 循环调用: OTP 序列跑完就主动上报结果
void CANSimple::as5047_otp_report() {
    if (!as5047_otp_pending) {
        return;
    }

    uint16_t out_zpos = 0xffffu;
    uint8_t fail = 0xff;
    if (!axes[0].encoder_.as5047_burn_otp_poll(&out_zpos, &fail)) {
        return; // 还在跑
    }

    as5047_otp_pending = false;

    if (fail == 0) {
        // 硬件零位已经永久变成 X, 电气 0 处计数归零, 故软件侧等效偏移必须为 0
        axes[0].encoder_.config_.phase_offset = 0;
        axes[0].encoder_.config_.phase_offset_float = 0.5f;
        axes[0].encoder_.config_.pre_calibrated = true;
        as5047_volatile_touched = true;
    }

    send_count_reply(0x44, out_zpos, fail);
}

// 0x277: 识别/校准命令, buf[1..7] 必须为 0x11..0x77
void CANSimple::identify_callback(const can_Message_t& msg) {
    if (msg.len < 8) {
        return;
    }
    if (!((msg.buf[1] == 0x11) && (msg.buf[2] == 0x22) && (msg.buf[3] == 0x33) &&
          (msg.buf[4] == 0x44) && (msg.buf[5] == 0x55) && (msg.buf[6] == 0x66) &&
          (msg.buf[7] == 0x77))) {
        return;
    }

    g_identify_mode = msg.buf[0];
    switch (g_identify_mode) {
        case 1:
            set_brake(0, true);  // 开闸便于校准
            set_brake(1, true);
            axes[0].requested_state_ = Axis::AXIS_STATE_MOTOR_CALIBRATION;
            break;
        case 2:
            axes[1].requested_state_ = Axis::AXIS_STATE_MOTOR_CALIBRATION;
            break;
        case 3:
            axes[0].requested_state_ = Axis::AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
            break;
        case 4:
            axes[1].requested_state_ = Axis::AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
            break;
        case 5:
            for (auto& axis : axes) {
                axis.motor_.config_.pre_calibrated = true;
                axis.encoder_.config_.pre_calibrated = true;
                axis.requested_state_ = Axis::AXIS_STATE_IDLE;
            }
            if ((axes[0].error_ == Axis::ERROR_NONE) || (axes[1].error_ == Axis::ERROR_NONE)) {
                odrv.save_configuration();
                odrv.reboot();
            }
            break;
        case 6:
            odrv.erase_configuration();//第一步需先擦掉配置，防止数据混乱,擦除后会复位            
            break;
        default:
            break;
    }
}

// 0x701: OTA 复位, 扩展帧 0x701 + 数据 04 31 01 02 03 -> 复位由 bootloader 接管升级
void CANSimple::handle_ota_message(const can_Message_t& msg) {
    if (!msg.isExt || (msg.len < 5)) {
        return;
    }
    if (!((msg.buf[0] == 0x04) && (msg.buf[1] == 0x31) && (msg.buf[2] == 0x01) &&
          (msg.buf[3] == 0x02) && (msg.buf[4] == 0x03))) {
        return;
    }
    NVIC_SystemReset();
}

/* ------------------------------------------------------------------------- */
/* 辅助                                                                       */
/* ------------------------------------------------------------------------- */

// 抱闸操作沿用 main.cpp 里已有的定义 (见 main.cpp: 抱闸控制 axis0-gpio8 axis1-gpio5):
//   轴0 = GPIO8, 轴1 = GPIO5, 高电平开闸 / 低电平抱闸
//   手动抱闸检测: 轴0 = GPIO3, 轴1 = GPIO4 (输入上拉, 高电平表示手动抱闸未解除)
void CANSimple::set_brake(size_t i, bool release) {
    if (i >= AXIS_COUNT) {
        return;
    }
    brake_released_[i] = release;
    get_gpio(i == 0 ? 8 : 5).write(release);
}

void CANSimple::stop_axes() {
    for (auto& axis : axes) {
        axis.controller_.input_vel_ = 0.0f;
        axis.requested_state_ = Axis::AXIS_STATE_IDLE;
    }
}

/* ------------------------------------------------------------------------- */
/* 发送                                                                       */
/* ------------------------------------------------------------------------- */

// 0x270/0x271 轴错误
bool CANSimple::send_axis_error(Axis& axis, uint32_t msg_id) {
    can_Message_t txmsg;
    txmsg.id = (&axis == &axes[0]) ? msg_id : (msg_id + 1);
    txmsg.isExt = true;
    txmsg.len = 8;

    txmsg.buf[0] = 0x20;  // MM 模块编号 (20-驱动板)
    txmsg.buf[1] = (&axis == &axes[0]) ? 0x07 : 0x08;  // 子模块: 轴0/轴1
    txmsg.buf[2] = 0x09;
    txmsg.buf[3] = (axis.error_ != Axis::ERROR_NONE) ? 0x01 : 0x02;  // 发生/解除
    const uint32_t error = static_cast<uint32_t>(axis.error_);
    txmsg.buf[4] = error >> 24;
    txmsg.buf[5] = error >> 16;
    txmsg.buf[6] = error >> 8;
    txmsg.buf[7] = error;

    return canbus_->send_message(txmsg);
}

// 0x272/0x273 电机错误
bool CANSimple::send_axis_motor_error(Axis& axis, uint32_t msg_id) {
    can_Message_t txmsg;
    txmsg.id = (&axis == &axes[0]) ? msg_id : (msg_id + 1);
    txmsg.isExt = true;
    txmsg.len = 8;

    txmsg.buf[0] = 0x20;
    txmsg.buf[1] = (&axis == &axes[0]) ? 0x09 : 0x0A;  // 电机0/电机1
    txmsg.buf[2] = 0x09;
    txmsg.buf[3] = (axis.motor_.error_ != Motor::ERROR_NONE) ? 0x01 : 0x02;
    const uint32_t error = static_cast<uint32_t>(axis.motor_.error_);
    txmsg.buf[4] = error >> 24;
    txmsg.buf[5] = error >> 16;
    txmsg.buf[6] = error >> 8;
    txmsg.buf[7] = error;

    return canbus_->send_message(txmsg);
}

// 0x275/0x276 编码器错误
bool CANSimple::send_encoder_error(Axis& axis, uint32_t msg_id) {
    can_Message_t txmsg;
    txmsg.id = (&axis == &axes[0]) ? msg_id : (msg_id + 1);
    txmsg.isExt = true;
    txmsg.len = 8;

    txmsg.buf[0] = 0x20;
    txmsg.buf[1] = (&axis == &axes[0]) ? 0x0B : 0x0C;  // 编码器0/编码器1
    txmsg.buf[2] = 0x09;
    txmsg.buf[3] = (axis.encoder_.error_ != Encoder::ERROR_NONE) ? 0x01 : 0x02;
    const uint32_t error = static_cast<uint32_t>(axis.encoder_.error_);
    txmsg.buf[4] = error >> 24;
    txmsg.buf[5] = error >> 16;
    txmsg.buf[6] = error >> 8;
    txmsg.buf[7] = error;

    return canbus_->send_message(txmsg);
}

// 0x21A/0x21E 累积编码器计数 (64 位, 大端), 用于上控里程统计
bool CANSimple::send_encoder_count(Axis& axis, uint32_t msg_id) {
    static int64_t accumulated_count[AXIS_COUNT] = {};
    static int32_t last_shadow_count[AXIS_COUNT] = {};
    static bool first_time[AXIS_COUNT] = {true, true};

    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        int32_t current = axes[i].encoder_.shadow_count_;
        if (first_time[i]) {
            last_shadow_count[i] = current;
            first_time[i] = false;
            continue;
        }
        // 用无符号差值自动处理 int32 溢出
        int32_t delta = static_cast<int32_t>(static_cast<uint32_t>(current) -
                                             static_cast<uint32_t>(last_shadow_count[i]));
        accumulated_count[i] += delta;
        last_shadow_count[i] = current;
    }

    const bool is_axis0 = (&axis == &axes[0]);
    int64_t count = accumulated_count[is_axis0 ? 0 : 1];
#ifdef ENCODER_USE_SPI_ABS_AMS
    // 磁编: 轴0 方向与上控约定相反
    if (is_axis0) {
        count = -count;
    }
#endif

    can_Message_t txmsg;
    txmsg.id = msg_id;
    txmsg.isExt = true;
    txmsg.len = 8;
    txmsg.buf[0] = count >> 56;
    txmsg.buf[1] = count >> 48;
    txmsg.buf[2] = count >> 40;
    txmsg.buf[3] = count >> 32;
    txmsg.buf[4] = count >> 24;
    txmsg.buf[5] = count >> 16;
    txmsg.buf[6] = count >> 8;
    txmsg.buf[7] = count;

    return canbus_->send_message(txmsg);
}

// 0x21C 左右轮速度反馈 (turn/s, 两个 float)
bool CANSimple::send_vel_estimate() {
    can_Message_t txmsg;
    txmsg.id = MSG_TX_VEL_ESTIMATE;
    txmsg.isExt = true;
    txmsg.len = 8;

    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        float vel = axes[i].encoder_.vel_estimate_.any().value_or(0.0f);
        if (std::fabs(vel) < 0.1f) {
            vel = 0.0f;
        } else {
#ifdef ENCODER_USE_SPI_ABS_AMS
            if (i == 0) {
                vel = -vel;
            }
#endif
        }
        can_setSignal<float>(txmsg, vel, (uint8_t)(32 * i), 32, true);
    }

    return canbus_->send_message(txmsg);
}

// 0x274 心跳/抱闸状态 (每 500ms 发一帧), 同时承担指令超时保护
bool CANSimple::send_heartbeat() {
    // 未完成识别前打开抱闸, 避免校准被机械闸卡住
    if (!(axes[0].encoder_.config_.pre_calibrated && axes[1].encoder_.config_.pre_calibrated)) {
        set_brake(0, true);//开闸
        set_brake(1, true);
    } else {
        msg_lost_count_++;

        // 指令超时: 速度清零
        if (msg_lost_count_ > MAX_MSG_LOST_COUNT) {
            for (auto& a : axes) {
                if (a.controller_.input_vel_ != 0.0f) {
                    a.controller_.input_vel_ = 0.0f;
                }
            }
        }

        // 指令超时且已停稳: 抱闸并切 IDLE
        if ((msg_lost_count_ > MAX_MSG_LOST_COUNT) && !back_push_sts_ && (g_identify_mode == 0)) {
            for (size_t i = 0; i < AXIS_COUNT; ++i) {
                Axis& a = axes[i];
                const float vel_est = a.encoder_.vel_estimate_.any().value_or(0.0f);
                if ((std::fabs(vel_est) <= 0.01f) && (a.controller_.input_vel_ == 0.0f)) {
                    if (a.requested_state_ == Axis::AXIS_STATE_CLOSED_LOOP_CONTROL) {
                        set_brake(i, false);  // 抱闸
                        a.requested_state_ = Axis::AXIS_STATE_IDLE;
                    }
                }
            }
        }
    }

    // 任一轴有错误时抱闸 (推行模式除外)
    for (auto& a : axes) {
        if (a.error_ != Axis::ERROR_NONE) {
            const bool release = (g_operate_mode == 3);
            set_brake(0, release);
            set_brake(1, release);
            break;
        }
    }

    // 手动抱闸检测 (GPIO3=轴0, GPIO4=轴1, 输入上拉), 带滤波防抖
    static uint16_t cnt_locked = 0;
    static uint16_t cnt_released = 0;
    const bool axis0_brake_sts = get_gpio(3).read();  // 1-手动抱闸未解除
    const bool axis1_brake_sts = get_gpio(4).read();
    if (axis0_brake_sts && axis1_brake_sts) {
        if (cnt_locked < BRAKE_FILTER_COUNT + 1) cnt_locked++;
        if (cnt_locked > BRAKE_FILTER_COUNT) {
            brake_sts_ = false;  // 手动抱闸未解除 -> 正常走车
            cnt_released = 0;
        }
    } else {
        if (cnt_released < BRAKE_FILTER_COUNT + 1) cnt_released++;
        if (cnt_released > BRAKE_FILTER_COUNT) {
            brake_sts_ = true;  // 手动抱闸已被人工解除 -> 禁止走车
            cnt_locked = 0;
        }
    }

    can_Message_t txmsg;
    txmsg.id = MSG_TX_HEARTBEAT;
    txmsg.isExt = true;
    txmsg.len = 8;
    txmsg.buf[0] = 0x20;  // MM 模块编号 (20-驱动板)
    txmsg.buf[1] = 0x02;  // SS 子模块编号 (02-抱闸)
    txmsg.buf[2] = 0x01;  // EE 子模块错误编号
    txmsg.buf[3] = brake_sts_ ? 0x01 : 0x02;  // 01-发生 02-解除
    txmsg.buf[4] = axes[0].encoder_.hall_state_;
    txmsg.buf[5] = axes[1].encoder_.hall_state_;
    txmsg.buf[6] = axis0_brake_sts ? 1 : 0;  // 1-手动抱闸未解除
    txmsg.buf[7] = axis1_brake_sts ? 1 : 0;

    // 心跳里顺带做霍尔错误自恢复
#ifdef ENCODER_USE_HALL
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        Axis& a = axes[i];
        if (a.encoder_.error_ == Encoder::ERROR_ILLEGAL_HALL_STATE) {
            if ((a.encoder_.hall_state_ != 0) && (a.encoder_.hall_state_ != 7)) {
                a.encoder_.error_ = Encoder::ERROR_NONE;
                a.error_ = Axis::ERROR_NONE;
            }
        }
    }
#endif

    // 参考工程: 内部 5 分频, 500ms 真正发一帧, 降低总线负载
    static uint8_t divider = 0;
    divider++;
    if (divider < 5) {
        return false;
    }
    divider = 0;

    return canbus_->send_message(txmsg);
}

/* ------------------------------------------------------------------------- */
/* 周期发送调度                                                               */
/* ------------------------------------------------------------------------- */

uint32_t CANSimple::service_stack() {
    const uint32_t now = HAL_GetTick();

    auto remaining = [now](uint32_t last, uint32_t period) -> uint32_t {
        const uint32_t elapsed = now - last;
        return (elapsed >= period) ? 0 : (period - elapsed);
    };

    // 每次 service 最多发 3 帧 (与参考工程一致, 避免总线突发);
    // 发满就留到下个周期, 各帧仍由自己的周期定时器限速。
    int budget = 3;

    // 0x21C 速度反馈 20ms
    if ((budget > 0) && ((now - axes[0].can_.last_encoder) >= VEL_ESTIMATE_PERIOD_MS)) {
        if (send_vel_estimate()) {
            axes[0].can_.last_encoder = now;
            budget--;
        }
    }

    // 0x270/0x271 轴错误
    if ((budget >= 2) && ((now - axes[0].can_.last_axis_error) >= AXIS_ERROR_PERIOD_MS)) {
        send_axis_error(axes[0], MSG_TX_AXIS_ERROR);
        send_axis_error(axes[1], MSG_TX_AXIS_ERROR);
        axes[0].can_.last_axis_error = now;
        budget -= 2;
    }

    // 0x272/0x273 电机错误
    if ((budget >= 2) && ((now - axes[0].can_.last_motor_error) >= MOTOR_ERROR_PERIOD_MS)) {
        send_axis_motor_error(axes[0], MSG_TX_AXIS_MOTOR_ERROR);
        send_axis_motor_error(axes[1], MSG_TX_AXIS_MOTOR_ERROR);
        axes[0].can_.last_motor_error = now;
        budget -= 2;
    }

    // 0x275/0x276 编码器错误
    if ((budget >= 2) && ((now - axes[0].can_.last_encoder_error) >= ENCODER_ERROR_PERIOD_MS)) {
        send_encoder_error(axes[0], MSG_TX_ENCODER_ERROR);
        send_encoder_error(axes[1], MSG_TX_ENCODER_ERROR);
        axes[0].can_.last_encoder_error = now;
        budget -= 2;
    }

    // 0x21A/0x21E 累积编码器计数
    if ((budget >= 2) && ((now - axes[0].can_.last_encoder_count) >= ENCODER_COUNT_PERIOD_MS)) {
        send_encoder_count(axes[0], MSG_TX_ENCODER_COUNT_AXIS0);
        send_encoder_count(axes[1], MSG_TX_ENCODER_COUNT_AXIS1);
        axes[0].can_.last_encoder_count = now;
        budget -= 2;
    }

    // 0x274 心跳/抱闸状态
    if ((budget > 0) && ((now - axes[0].can_.last_heartbeat) >= HEARTBEAT_PERIOD_MS)) {
#ifndef DEBUG_USB
        send_heartbeat();
#endif
        axes[0].can_.last_heartbeat = now;
        budget--;
    }

    // AS5047 OTP 序列跑完则主动上报结果 (非阻塞设计的另一半: 发起在 0x44 里, 结果在这里查)
    as5047_otp_report();

    uint32_t next_service = remaining(axes[0].can_.last_encoder, VEL_ESTIMATE_PERIOD_MS);
    next_service = std::min(next_service, remaining(axes[0].can_.last_axis_error, AXIS_ERROR_PERIOD_MS));
    next_service = std::min(next_service, remaining(axes[0].can_.last_motor_error, MOTOR_ERROR_PERIOD_MS));
    next_service = std::min(next_service, remaining(axes[0].can_.last_encoder_error, ENCODER_ERROR_PERIOD_MS));
    next_service = std::min(next_service, remaining(axes[0].can_.last_encoder_count, ENCODER_COUNT_PERIOD_MS));
    next_service = std::min(next_service, remaining(axes[0].can_.last_heartbeat, HEARTBEAT_PERIOD_MS));

    return std::max(next_service, 1UL);
}

/* ------------------------------------------------------------------------- */
/* 版本信息 (强制放到指定 Flash 地址)                                          */
/* ------------------------------------------------------------------------- */

typedef struct {
    uint32_t software_version;  // 软件版本
    uint32_t publish_data;      // 发布日期
} VersionInfo;

__attribute__((section(".version_info"), used))
const VersionInfo version_info = {
    .software_version = 0x0108,
    .publish_data = 0x20261020,
};

uint16_t get_hardware_version(void) {
    return *(volatile uint16_t*)(0x08000188);
}
 
uint32_t get_software_version(void) {
    return *(volatile uint32_t*)0x0800C188;
}

uint32_t get_publish_data(void) {
    return *(volatile uint32_t*)(0x0800C188 + 4);
}

/* ------------------------------------------------------------------------- */
/* 已停用: 标准 ODrive "nodeID|cmdID" 命令解析                                 */
/* ------------------------------------------------------------------------- */

// do_command() 已停用: 原来按 (node_id << 5 | cmd_id) 解析节点的命令,
// 现在改为固定 CAN ID, 直接在 handle_can_message() 的 switch (msg.id) 里分发。
//
// 原流程 (保留备查):
//   for (auto& axis : axes) {
//       if ((axis.config_.can.node_id == node_id) &&
//           (axis.config_.can.is_extended == msg.isExt)) {
//           do_command(axis, msg);
//           return;
//       }
//   }
//   do_command() 内部: case MSG_SET_AXIS_REQUESTED_STATE / MSG_SET_INPUT_VEL /
//   MSG_GET_ENCODER_ESTIMATES / ... 等 cmd_id 分支, 均已由上面的 switch 取代。
