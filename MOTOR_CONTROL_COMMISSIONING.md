# JD-Power MY-4625F 电机控制调试记录

> ⚠️ **阅读提示**
> 本文上半部分（到「下一步建议」为止）记录的是当时的调试过程，其中
> **“励磁时编码器被磁场/EMI 干扰”这一结论已被推翻**。
> 真正根因与最终结果见文末 **[复盘：真正根因不是 EMI，已闭环]** 一节。
> 目前 axis0 已可正常闭环运行。

## 目标对象

- 电机型号：JD-Power MY-4625F 4012 外转电机
- 结构信息：24N22P
- KV：55 rpm/V
- 标称电压范围：10V ~ 24V
- 最大扭矩：2.5 kgf-cm
- 编码器：AS5048A SPI 绝对编码器
- ODrive 轴：axis0

## 当前结论

电机参数配置和电机本体电气测量已经基本打通，但 **FOC 闭环目前仍未完成**。

阻塞点不在电机参数本身，而在 **电机励磁时编码器读数失真，导致 encoder offset calibration 失败**。

换句话说：

- 手动转动时，AS5048A 通信正常
- 电机一进入带电校准，编码器计数就出现大幅跳变
- 因此 ODrive 无法建立可靠的电角度偏移
- 所以暂时还不能进入真正可用的闭环控制

## 本次使用的关键参数

已确认正确或较合理的基础配置：

- `motor.config.motor_type = 0` (`MOTOR_TYPE_HIGH_CURRENT`)
- `motor.config.pole_pairs = 11`
- `motor.config.torque_constant = 8.27 / 55 = 0.15036`
- `encoder.config.mode = 257` (`AMS SPI ABS`)
- `encoder.config.cpr = 16384`
- `encoder.config.abs_spi_cs_gpio_pin = 7`

## 已验证的基础状态

在未进入电机带电校准前：

- `axis.error = 0`
- `motor.error = 0`
- `encoder.error = 0`
- 手动转动时 `pos_abs` 正常更新
- 连续采样期间未再复现 `ERROR_ABS_SPI_COM_FAIL (128)`

这说明此前的 SPI 通信问题已经解决。

## 电机参数配置结果

已经把以下基础参数写入并验证过：

- `pole_pairs = 11`
- `torque_constant = 0.15036364`
- `control_mode = POSITION_CONTROL`
- `input_mode = PASSTHROUGH`

其中 `pole_pairs` 和 `torque_constant` 与电机铭牌信息是一致的。

## 电机校准阶段的结果

### 第一阶段：高电流配置失败

最初使用较大的 `calibration_current` 时，校准会在 motor calibration 早期失败，表现为：

- `MOTOR_ERROR_PHASE_RESISTANCE_OUT_OF_RANGE`
- `phase_resistance = nan`

这表明目标校准电流过高，控制器为了达到该电流，测试电压积分到了上限仍不够，因此不是“电机断路”，而是参数对这台低电流电机不合适。

### 第二阶段：降低校准电流后，motor calibration 成功

当参数调整为更保守组合后，motor calibration 成功给出了稳定测量值：

- `calibration_current = 1.0 A`
- `current_lim = 2.0 A`
- `resistance_calib_max_voltage = 4.0 V`

测得的电机参数为：

- `phase_resistance ≈ 2.1809 ohm`
- `phase_inductance ≈ 0.0005823 H`

这说明：

- ODrive 对这台电机的电气建模已经开始有效
- 电机不是完全无法驱动
- 真正卡住的是后续的 encoder offset calibration

## 编码器偏移校准阶段的结果

在 motor calibration 成功后，full calibration 会继续进入：

- `AXIS_STATE_ENCODER_OFFSET_CALIBRATION` (`state = 7`)

该阶段最终失败，错误为：

- `axis.error = 256` (`AXIS_ERROR_ENCODER_FAILED`)
- `encoder.error = 2` (`ENCODER_ERROR_CPR_POLEPAIRS_MISMATCH`)

## 为什么这个错误不代表 pole_pairs 一定填错了

ODrive 在 offset calibration 中会比较：

- 编码器实际计数变化量
- 根据 `pole_pairs`、`cpr` 和扫描距离推算出的理论变化量

对当前参数：

- `pole_pairs = 11`
- `cpr = 16384`
- `calib_scan_distance = 16π electrical rad`

理论上，编码器响应量大约应为：

- `8 * cpr / pole_pairs ≈ 11915 counts`

但实际测到：

- `calib_scan_response = 276238` 到 `289893 counts`

这个偏差不是“11 写成了 10 或 12”这种级别，而是差了一个数量级以上，因此更像：

- 电机励磁时编码器计数大范围跳变
- 导致 `shadow_count` 在校准过程中疯狂累计
- 最终触发 `CPR_POLEPAIRS_MISMATCH`

## 直接证据：只跑 encoder offset calibration 时的采样结果

单独执行 `AXIS_STATE_ENCODER_OFFSET_CALIBRATION` 并高频采样后得到：

- `states_seen = [1, 7]`
- `pos_min = 243`
- `pos_max = 15348`
- `pos_distinct = 279`
- `max_pos_step = 14848`
- `shadow_min = -94366`
- `shadow_max = 45132`
- `max_shadow_step = 90376`

这说明在电机带电扫描时：

- `pos_abs` 存在接近整圈量级的跳变
- `shadow_count` 出现远超正常机械运动的突变
- 编码器虽然没有报 SPI 通信错误，但位置本身在励磁环境下已经不可信

## 当前最合理的工程判断

当前最合理的判断是：

### 1. AS5048A 的静态通信问题已经解决

也就是此前的 `ERROR_ABS_SPI_COM_FAIL (128)` 已经不是当前主问题。

### 2. 电机励磁磁场或 PWM 干扰正在影响编码器角度测量

这可能来自：

- 电机定子电流产生的附加磁场干扰 AS5048A
- 编码器安装位置离励磁区域过近
- 编码器电缆在电机 PWM 驱动时受到干扰
- 当前 32.5V 母线电压过高，导致 PWM 边沿更强、干扰更重

### 3. 当前真正阻塞闭环的是“励磁时角度不可信”

只要 offset calibration 无法稳定完成，就不能认为 FOC 闭环已经建立成功。

## 特别注意：当前母线电压偏高

实测 `vbus ≈ 32.5V`，已经高于这台电机的标称 10V ~ 24V 范围。

虽然目前测试都限制在低电流、低速度、校准场景，但这仍然是一个重要风险：

- 不适合继续做高速或大扭矩测试
- 也可能加剧编码器或布线的电磁干扰问题

## 12V 降压后的复测结果

在把母线电压降到约 `12.37V` 后，重新做了校准相关测试。

### 改善点

与 32V 级别母线相比，12V 下出现了明显改善：

- 不再触发此前的系统级回灌问题
- motor calibration 仍然可以稳定测出电机参数
- 励磁干扰的量级相比高压时有所减弱

在 12V 条件下测得：

- `phase_resistance ≈ 1.995Ω`
- `phase_inductance ≈ 0.000730H`

这些数值与之前同量级，说明电机电气测量结果是可信的。

### 仍然存在的问题

即使降到 12V，encoder offset calibration 仍未完成，仍然报：

- `axis.error = 256`
- `encoder.error = 2` (`ENCODER_ERROR_CPR_POLEPAIRS_MISMATCH`)

但这次数据表明问题程度有所减轻，而不是彻底消失：

- 高压条件下曾出现 `calib_scan_response ≈ 276238 ~ 309003`
- 12V、低励磁尝试时最低降到 `calib_scan_response ≈ 75546`

理论期望值大约仍只有：

- `≈ 11915 counts`

这说明：

- 降压确实降低了干扰强度
- 但励磁下编码器计数仍然严重失真
- 当前还不足以完成可靠的电角度偏移校准

### 当前判断更新

12V 供电验证说明一个重要事实：

- **母线过压会加剧问题，但不是唯一根因。**

更本质的 blocker 仍然是：

- 电机励磁时，AS5048A 的角度测量仍受到明显磁场或 EMI 干扰

## 继续降压到约 5.3V 的结果

后续继续把母线压低到约 `5.31V` 后，问题不再是 encoder offset calibration，而是系统已经进入欠压工作区：

- `LegacyODriveError.DC_BUS_UNDER_VOLTAGE`
- `MotorError.DRV_FAULT`
- `MotorError.SYSTEM_LEVEL`

对应现象：

- `vbus ≈ 5.31V`
- `axis.error = 64`
- `motor.error = 16777224 = DRV_FAULT | SYSTEM_LEVEL`
- 编码器本身未报新错误

这说明：

- 再继续降压并不能越过原来的 encoder mismatch 问题
- 反而会先把 ODrive 功率级带入欠压/驱动器故障区

因此，当前可用的测试窗口应理解为：

- 过高电压会加剧励磁干扰
- 过低电压会触发欠压和 DRV fault
- 后续更合理的调试区间应维持在电机标称范围内，例如 `10V ~ 12V` 左右，而不是继续往下压

## 调整安装后的复测结果

在重新调整编码器安装之后，又在约 `11.37V` 母线下做了一轮复测。

### 静态状态

静态条件下基线是干净的：

- `axis.error = 0`
- `motor.error = 0`
- `encoder.error = 0`
- 静态采样时编码器位置只在很小范围内微动

这说明安装调整没有破坏原有的静态 SPI 通信稳定性。

### full calibration 结果

使用较低风险参数：

- `calibration_current = 0.5`
- `current_lim = 1.0`
- `resistance_calib_max_voltage = 4.0`
- `encoder.config.calib_range = 0.20`
- `encoder.config.calib_scan_omega = 1.0`

full calibration 的过程为：

- 进入 `state 4`：motor calibration
- 成功进入 `state 7`：encoder offset calibration
- 在大约 `55s` 后失败并返回 idle

最终结果：

- `axis.error = 256`
- `motor.error = 0`
- `encoder.error = 2`
- `encoder_ready = False`
- `phase_resistance ≈ 2.055Ω`
- `phase_inductance ≈ 0.000693H`
- `calib_scan_response ≈ 231731`

### 结论更新

安装调整后，确实出现了一定改善：

- 校准能更稳定地进入 encoder offset calibration
- 过程持续时间更长
- 相比最差状态，干扰强度有所下降

但目前仍然没有跨过根本阻塞：

- encoder offset calibration 最终仍以 `ENCODER_ERROR_CPR_POLEPAIRS_MISMATCH` 结束
- `calib_scan_response` 仍远大于理论期望值

所以截至当前，合理结论是：

- **安装调整带来了改善，但还不足以让 FOC 所需的电角度偏移校准成功。**

因此，下一步重点仍应放在：

- 编码器安装位置
- 磁路隔离
- 供电退耦
- SPI 布线与电机相线分离

而不是继续单纯反复调整 ODrive 校准参数。

## 下一步建议

按优先级推荐如下：

1. 先把母线电压降到电机额定范围内，例如 12V ~ 24V，再重复 encoder offset calibration。
2. 优化 AS5048A 安装与布线：增加与电机励磁区域的距离，改善地线与供电退耦，SPI 线与电机三相线分离。
3. 若条件允许，使用示波器或逻辑分析仪在电机带电校准时同时观察 `CS/SCK/MISO` 与电机相线环境。
4. 如果磁场耦合无法避免，考虑更换为更抗干扰的编码器方案，或更改机械安装方式。
5. 在确认编码器励磁下仍稳定之前，不建议把当前状态视为“已实现可用的 FOC 闭环控制”。

## 一句话总结

目前电机电气参数已经能测出来，但 encoder offset calibration 失败；根因高度怀疑是 **电机励磁时 AS5048A 角度测量被磁场/EMI 干扰**，因此当前还不能宣称 axis0 已完成可用的 FOC 闭环控制。
---

# 复盘（后续更新）：真正根因不是 EMI，已闭环

上面“励磁时编码器失真”的结论是在**有 bug 的固件**上得出的，现已被推翻。
axis0 现在可以正常闭环转动，上电即用。

## 真正的原因链

### 1. SPI 相位错了，导致整帧错位

`Encoder::setup()` 里 AMS 分支的 `CLKPhase` 曾被改成 `SPI_PHASE_1EDGE`（commit `78035e40`），
但 AS5048A 是 **SPI mode 1（CPOL=0, CPHA=1）**，必须用 `SPI_PHASE_2EDGE`。
1EDGE 会让主机早采一个时钟，整帧左移一位，偶校验几乎必然失败。

### 2. “奇偶位重建”把错误藏起来了

后续的 commit（`8c7d458f` 等）在解析前用移位后的值**重新计算并写回奇偶位**，
于是校验变成自证的、永远通过，任意错误帧都会被当成有效角度。
表现就是电机静止时 `pos_abs` 在 5453 / 11541 / 5613 / 3349 之间乱跳。

这就是 `calib_scan_response ≈ 231731`（理论 11915）的真正来源：
不是磁场干扰，是解析器在编数据，`shadow_count` 被垃圾增量累加爆掉。

### 3. 修好相位后暴露的真问题：AS5048A 的 Error Flag 常 1

改成 2EDGE + 严格偶校验后，实测 139832 帧里只有 2 帧奇偶失败——
说明时序和链路都是好的，原始帧稳定在 `0x7455/0xF454`（位置 13397）。

但**每一帧的 bit14（磁场 Error Flag）都是 1**，固件按设计丢弃所有帧，
于是 `pos_abs` 恒为 0、`encoder.error = 128`。

人工转轴实测：角度连续、满量程 99.5%、噪声带 7–15 counts（约 0.2–0.3°），
说明**角度是真实可用的，只是芯片认为磁场超出规格**。

## 最终修复

- `CLKPhase` 改回 `SPI_PHASE_2EDGE`（SPI mode 1），SPI 时钟 2.6 MHz
- AMS 解析恢复严格偶校验 + EF 检查，删除移位/字节交换/重算奇偶的所有补丁
- 保留 `abs_spi_transaction_pending_`（在途 DMA 事务不计失败）
- 新增持久配置 `encoder.config.abs_spi_ignore_error_flag`，默认 `false`；
  本板置 `true`，因为 EF 常 1 但角度可信
- 移除中途加的 0x7FFE/0x7FFD 探针（回调与下一帧启动存在竞态，会把寄存器值当成角度）

相关提交：`096ad126`、`b7b47562`、`4b34d02c`、`87fde1ae`、`4390a767`、`35ed8e72`

## 最终验证结果

```
motor:   cal=True pre_cal=True pp=11 R=1.9396Ω L=0.690mH Ilim=3.0A
encoder: err=0 ready=True mode=257 cpr=16384 cs_gpio=7
         offset_float=0.6273 direction=1 ignore_ef=True
ctrl:    control_mode=2 velocity, input_mode=2 vel_ramp, vg=0.08 vi=0.4 ramp=2.0
```

- encoder offset calibration：`calib_scan_response = 11889`（期望 11916，误差 0.2%）
- 闭环速度环（VEL_RAMP，加速度限制 2 turn/s²）：

| 目标 | 实测均值 | 范围 | 错误 |
|------|----------|------|------|
| +1.0 turn/s | +0.996 | 0.78 ~ 1.21 | 全 0 |
| +2.0 turn/s | +2.024 | 1.71 ~ 2.24 | 全 0 |
| -1.0 turn/s | -0.721（换向窗口） | -1.23 ~ +0.05 | 全 0 |
| 0 | +0.004 | -0.07 ~ +0.07 | 全 0 |

全部配置已 `save_configuration()`，**断电重启后无需任何脚本即可闭环运行**。

## 仍然遗留的硬件问题

AS5048A 的 Error Flag 常 1，说明磁铁仍不在规格内（气隙 / 同心度 / 充磁方向）。
角度虽然可用，但这是治标。建议：

1. 确认磁铁是**径向（diametric）充磁**的圆片/圆柱磁铁，不是轴向充磁、不是多极磁环
2. 磁铁中心与芯片中心对齐，气隙 1–2 mm
3. 换好磁铁后，把 `encoder.config.abs_spi_ignore_error_flag` 改回 `false` 并保存，
   确认 `encoder.error` 仍为 0，此时才算真正修好

## 调参经验

- 这台 55KV 小电机摩擦很小，默认增益（vel_gain 0.167 / vel_integrator_gain 0.333）
  在速度环里会一次给到 0.05 N·m 级别扭矩，25 ms 内就冲过 vel_limit 触发 OVERSPEED。
  最终用 `vel_gain=0.08 / vel_integrator_gain=0.4`。
- 速度环**必须**配 `input_mode = VEL_RAMP` + `vel_ramp_rate = 2.0`，
  否则阶跃给定 + 3 A 电流限幅会让电机猛地一冲然后过速保护。
- `enable_brake_resistor` 只在**上电时**生效：运行中再置 true 会报 `BRAKE_RESISTOR_DISARMED`，
  必须保存配置后重启。刹车电阻实际阻值也要和 `config.brake_resistance` 一致。
