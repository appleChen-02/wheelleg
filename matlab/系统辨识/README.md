# 系统辨识模式（无激励）数据分析

## 文件

- `analyze_sysid_no_excitation.m` — 主分析脚本（MATLAB R2024）

## 数据准备

1. 用 `上位机/usb_data_logger.py` 在**系统辨识模式**下采集（`SYS_ID_EXCITATION_ENABLE=0`，双足模式保持工作点）；
2. 把采集目录（至少包含以下两个文件）放到本目录的 `data/` 子目录下，或修改脚本头部 `cfg.dataDir` 指向采集目录：
   - `system_identification_12.csv`（0x10 高速帧，250Hz）
   - `system_identification_reference.csv`（0x12 参考配置帧，1Hz）

## 运行

```matlab
cd matlab/系统辨识
analyze_sysid_no_excitation
```

如需手动截取稳态区间，设置 `cfg.tRange = [t0 t1]`（单位 s，相对文件首样本）后重新运行；
默认使用整个文件（记录内容应已是选定的稳态段）。

## 输出

结果写入 `results/<时间戳>/`：

| 文件 | 内容 |
| --- | --- |
| summary.csv | 采集质量、采样率、丢帧率、激励/状态标志统计等关键值 |
| operating_point.csv | 12 状态工作点均值 + 5 路 LQR/command/actual 均值 |
| state_bias.csv | 实测工作点 − 参考工作点偏差 |
| state/input_statistics.csv | 各通道 mean/median/样本标准差/AC RMS/peak-to-peak/P99-P1/MAD/min/max |
| drift_statistics.csv | body_x、yaw、body_x_dot、yaw_dot 漂移斜率 + 95% 置信区间 + 二次项显著性 |
| state/input_covariance.csv, cross_covariance.csv | 稳态协方差与相关矩阵 |
| k_check.csv | Cov(u_lqr) ≈ K·Cov(error)·K′ 一致性检验（K 取自固件 K_coef_Bipedal @ l=0.18） |
| command_residual.csv | command − controller − equilibrium − excitation（应≈数值零） |
| psd_band_rms.csv | 0–1 / 1–20 / 20 Hz–Nyquist 三频段积分 RMS |
| data_quality.json | 全部质量信息与 issue 列表 |
| fig1~fig5*.png | 状态时域 / 输入时域 / 漂移拟合 / PSD / 相关矩阵 |

命令行同时打印最终结论（工作点、偏差、漂移率、稳态波动、LQR 偏置与波动、K 一致性、注意事项）。

## 注意

- 时间轴使用 `control_tick_ms`，`host_time_s` 仅作参考对比；
- 记录内容视为已选定的稳态段，脚本不再做预热切除或稳态自动检测；
- 本分析是后续激励实验的基线，**不能**单独估计纯传感器测量噪声——
  如需估计测量噪声，请另采一组"机器人固定 + 执行器关闭/机械锁止"的对照数据。
