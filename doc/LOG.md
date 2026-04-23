# 修改日志

## 2026-04-23

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
