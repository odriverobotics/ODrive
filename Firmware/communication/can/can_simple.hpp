#ifndef __CAN_SIMPLE_HPP_
#define __CAN_SIMPLE_HPP_

#include "canbus.hpp"
#include "axis.hpp"

/**
 * @brief 本工程自定义的 CAN 协议层 (移植自 odrive-vlago 工程)
 *
 * 接收 (全部为扩展帧, 由 init() 里的两个硬件过滤 bank 完成筛选):
 *   0x208  设置 PID (并回读 PID)
 *   0x217  获取母线电压/电流 (RTR)
 *   0x219  上控速度指令 (左右轮, 两个 float)
 *   0x220  上控速度指令 (与 0x219 同格式)
 *   0x221  运行模式 (模式 7 切换 PID)
 *   0x222  档位 1~4
 *   0x229  驾驶模式 1-舒适 2-运动
 *   0x21B  抱闸控制 (并回读抱闸状态)
 *   0x21D  获取版本信息
 *   0x277  识别/校准命令 (尾缀 11 22 33 44 55 66 77)
 *          或 磁编 AS5047P 零位标定 (尾缀 01 02 03 04 05 06 07, 见 PHASE_LOCK_REPLY_MSG_ID)
 *   0x279  摇杆 ADC 值 (本地摇杆控车, 差速/原地转圈模型)
 *   0x701  OTA 复位 (bootloader 接管)
 *
 * 发送 (扩展帧):
 *   0x217, 0x21A, 0x21C, 0x21D, 0x21E, 0x270~0x276, 0x278 (磁编零位标定应答)
 *
 * 说明: 原来 ODrive 标准的 "nodeID<<5 | cmdID" 命令解析 (do_command) 已停用,
 *       现在完全按上面的固定 CAN ID 分发, 见 can_simple.cpp 的 switch (msg.id)。
 */
class CANSimple {
   public:
    enum : uint32_t {
        // ---- 接收: 扩展帧 0x2xx 组 (bank0: id=0x200, mask=0x700) ----
        MSG_SET_AXIS_CONFIG_PID = 0x208,   // 设置 PID
        MSG_GET_VBUS_VOLTAGE = 0x217,      // 获取母线电压 (RTR)
        MSG_SET_VLAGO_INPUT_VEL = 0x219,   // 上控速度指令
        MSG_SET_VLAGO_BRAKE_VEL = 0x21B,   // 抱闸控制
        MSG_GET_VLAGO_VERSION = 0x21D,     // 获取版本信息
        MSG_SET_UP_CONTROLL_VEL = 0x220,   // 上控速度指令 (与 0x219 同格式)
        MSG_SET_OPERATE_MODE = 0x221,      // 运行模式 (切换 PID)
        MSG_SET_LEVEL = 0x222,             // 档位
        MSG_SET_DRIVE_MODE = 0x229,        // 驾驶模式 1-舒适 2-运动
        MSG_IDENTIFY = 0x277,              // 识别/校准命令
        MSG_SET_VLAGO_ADC = 0x279,         // 摇杆 ADC 值 (本地摇杆控车)

        // ---- 接收: 扩展帧精确匹配 (bank1) ----
        MSG_OTA_RESET = 0x701,            // OTA 复位

        // ---- 发送: 扩展帧 ----
        MSG_TX_ENCODER_COUNT_AXIS0 = 0x21A,  // 轴0 累积编码器计数
        MSG_TX_VEL_ESTIMATE = 0x21C,         // 左右轮速度反馈
        MSG_TX_ENCODER_COUNT_AXIS1 = 0x21E,  // 轴1 累积编码器计数
        MSG_TX_AXIS_ERROR = 0x270,           // 轴错误 (0x270/0x271)
        MSG_TX_AXIS_MOTOR_ERROR = 0x272,     // 电机错误 (0x272/0x273)
        MSG_TX_HEARTBEAT = 0x274,            // 心跳/抱闸状态
        MSG_TX_ENCODER_ERROR = 0x275,        // 编码器错误 (0x275/0x276)
    };

    CANSimple(CanBusBase* canbus) : canbus_(canbus) {}

    bool init();
    uint32_t service_stack();

   private:
    // 统一的接收回调: 一个 switch (msg.id) 分发所有命令
    static void can_message_callback(void* ctx, const can_Message_t& msg);
    void handle_can_message(const can_Message_t& msg);

    // ---- 接收命令处理 ----
    void set_axis_pid_config_callback(const can_Message_t& msg);   // 0x208
    void get_vbus_voltage_callback(const can_Message_t& msg);      // 0x217
    void set_vlago_input_vel_callback(const can_Message_t& msg);   // 0x219 / 0x220 / 0x279
    void set_vlago_brake_vel_callback(const can_Message_t& msg);   // 0x21B
    void set_operate_mode_callback(const can_Message_t& msg);      // 0x221
    void set_level_callback(const can_Message_t& msg);             // 0x222
    void set_drive_mode_callback(const can_Message_t& msg);        // 0x229
    void get_vlago_version_callback(const can_Message_t& msg);     // 0x21D
    void identify_callback(const can_Message_t& msg);              // 0x277
    void handle_ota_message(const can_Message_t& msg);             // 0x701

    // ---- 磁编 AS5047P 零位标定 / 电角度 0 锁定 / OTP 烧写: ID 0x277 (扩展帧), DLC = 8 ----
    //   byte0 = 0x00 : 读 AS5047 磁场强度与诊断信息, 应答 0x278
    //   byte0 = 0x11 : 锁定到电角度 0 → 读计数 X → 设好软件侧对齐参数, 应答 0x278
    //   byte0 = 0x22 : 退出锁定 (回到 IDLE, disarm), 无应答
    //   byte0 = 0x33 : 把 0x11 读到的 X 软写进 ZPOSM/ZPOSL (易失), 再读回计数, 应答 0x278
    //   byte0 = 0x44 : 把 0x11 读到的 X 烧进 OTP (不可逆, 只能烧一次), 应答 0x278
    //   byte1~byte7 = 01 02 03 04 05 06 07 固定校验字 (与 identify_callback 的尾缀区分)
    static constexpr uint32_t PHASE_LOCK_REPLY_MSG_ID = 0x278;
    void phase_lock_zero_callback(const can_Message_t& msg);      // 0x11 / 0x22
    void as5047_read_diag_callback(const can_Message_t& msg);     // 0x00
    void as5047_zero_offset_callback(const can_Message_t& msg);   // 0x33
    void as5047_burn_otp_callback(const can_Message_t& msg);      // 0x44
    void as5047_otp_report();                                     // OTP 结果上报 (service_stack 调用)
    // 0x278 应答: byte0 = 命令回显, byte1/2 = 14bit 数据 (byte1 高 8 位), byte3 = 结果码
    void send_count_reply(uint8_t cmd_echo, uint16_t count, uint8_t code = 0);
    // 0x278 诊断应答: byte0 = 0x00, byte1/2 = 磁场强度, byte3 = 诊断状态位, byte4 = AGC
    void send_diag_reply(uint8_t cmd_echo, uint16_t magnitude, uint8_t status, uint8_t agc);

    // ---- 发送 ----
    bool send_axis_error(Axis& axis, uint32_t msg_id);        // 0x270/0x271
    bool send_axis_motor_error(Axis& axis, uint32_t msg_id);  // 0x272/0x273
    bool send_encoder_error(Axis& axis, uint32_t msg_id);     // 0x275/0x276
    bool send_encoder_count(Axis& axis, uint32_t msg_id);     // 0x21A/0x21E
    bool send_vel_estimate();                                 // 0x21C
    bool send_heartbeat();                                    // 0x274

    // ---- 辅助 ----
    void set_brake(size_t i, bool release);
    void stop_axes();

    // 已停用: 标准 ODrive "nodeID|cmdID" 命令解析 do_command(),
    // 不再使用也不再声明, 原来的实现以注释形式保留在 can_simple.cpp 末尾。

    CanBusBase* canbus_;
    CanBusBase::CanSubscription* subscription_handle_ = nullptr;      // bank0: 0x2xx
    CanBusBase::CanSubscription* ota_subscription_handle_ = nullptr;  // bank1: 0x701

    static uint32_t msg_lost_count_;
    static bool brake_sts_;       // 0-手动抱闸不解除, 1-手动抱闸解除
    static bool back_push_sts_;   // 推行状态
    static bool brake_released_[AXIS_COUNT];  // 抱闸输出逻辑状态, true=已解除
};

#endif
