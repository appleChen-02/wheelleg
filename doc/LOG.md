# 修改日志

## 2026-04-23

### X/YAW 反馈与平衡偏置调试
- 修改 `application/robot_param_balanced_infantry.h`
- 将 `ENABLE_CHASSIS_X_POSITION_FEEDBACK` 调整为同时控制 x 与 yaw 的反馈策略
- 增加 `CHASSIS_X_POSITION_FEEDBACK_LIMIT` 与 `CHASSIS_YAW_FEEDBACK_LIMIT`，用于非全量反馈模式下对 x/yaw 误差做限幅
- 经实测后取消默认限幅，将 `ENABLE_CHASSIS_X_POSITION_FEEDBACK` 设为 `1`，默认使用全量 x/yaw 反馈
- 新增 `CHASSIS_X_BALANCE_OFFSET = -0.8f`，作为建模偏差补偿，叠加到 Pro 控制器的前向位移输入状态量中
- 保留 `X0_OFFSET` 原始用途，未直接将其改为 `-0.8f`，避免以后切换普通控制器时误伤不同状态量含义

### OLED 调试信息
- 修改 `application/other/oled_task.c`
- 修改 `application/chassis/chassis_balance.c/h`
- 在 OLED 上增加前向位移调试信息显示：`XF` 为反馈前向位移，`XR` 为参考前向位移
- 在 OLED 上增加误差显示：`XE` 为原始前向位移误差，`XI` 为实际输入控制器的前向位移状态量
- `XI` 当前包含 `CHASSIS_X_BALANCE_OFFSET`，便于现场确认 `-0.8f` 基础偏置是否真正进入控制器

### 编译修复
- 修改 `application/chassis/chassis_balance.c`
- 为 `WrapToPi()` 增加前置声明，修复新增 yaw helper 后 Keil 报出的隐式声明与 static 声明不兼容问题

### USB 控制模式调整
- 修改 `application/chassis/chassis_balance.c`
- 将遥控器模式开关下档固定映射为 `CHASSIS_JOINED`，不再要求 USB 在线后才能进入该模式
- 在 `CHASSIS_JOINED` 模式下增加 USB 断链兜底逻辑：当没有 USB 控制信号时，速度与角速度清零，姿态与腿部目标回到基础平衡值，用于维持基础站立平衡
- 调整该模式下夹爪目标的默认处理，避免 USB 模式断链时继续跟随 SBUS 通道产生附带动作

### 前向位移 x 反馈开关
- 修改 `application/robot_param_balanced_infantry.h`
- 新增参数 `ENABLE_CHASSIS_X_POSITION_FEEDBACK`，默认值为 `0`
- 该参数用于决定是否启用前向位移 `x` 位置反馈
- 默认关闭时，控制器保留速度相关反馈，但不再将 `x` 位置误差和 `Delta_x` 增量误差送入相关 LQR/MPC 状态量

### 控制器内部整理
- 修改 `application/chassis/chassis_balance.c`
- 为 `x` 位置反馈增加统一辅助函数，集中控制 body/leg/average leg 的位置误差与增量误差入口
- 将多个 locomotion controller 中直接使用 `x` 位置误差的地方统一改为走参数开关，方便后续继续调试和切换

### 模式拨杆映射调整
- 修改 `application/chassis/chassis_balance.c`
- 对调 `CHASSIS_MODE_CHANNEL` 的上档与中档映射关系
- 调整后，上档切换为原中档逻辑：`CHASSIS_FUNCTION` 下档时进入 `CHASSIS_BIPEDAL`，否则进入 `CHASSIS_TRIPOD`
- 调整后，中档切换为原上档逻辑：进入 `CHASSIS_NOTAIL`
- 下档保持不变，继续进入 `CHASSIS_JOINED`（USB 模式）
