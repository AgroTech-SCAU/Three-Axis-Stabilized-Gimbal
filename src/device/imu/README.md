# IMU 采集与姿态接口

## 职责

| 文件 | 职责 |
|---|---|
| `imu.h/.c` | 绑定设备实例并转发通用接口 |
| `bmi088.h/.c` | BMI088 SPI、DRDY、DMA、原始队列与采集快照 |
| `imu_attitude.h/.c` | 不访问硬件的六轴姿态算法 |
| `src/service/attitude.h/.c` | 独占原始队列消费、静止统计和姿态算法实例 |

`assemble_imu()` 绑定异步 `bmi088_instance`，关闭驱动内部融合 `IMU_ATTITUDE_NONE`，保留原采样率、DRDY/DMA、队列与缓存维护

应用通过 `attitude.get_snapshot()` 或 `attitude.get_state()` 读取结果，不并行调用 `imu_update()` 或启动第二套融合

## 坐标与时间

- 机体系为 X 前、Y 左、Z 上，采集不额外反转 X/Y
- 角度为 rad，角速度为 rad/s，加速度为 m/s²，温度为 ℃
- gyro、acc、temperature 使用同源微秒时钟与各自采样时间戳，acc 与 gyro 不是同步采样
- 积分使用 gyro 时间戳，`uint32_t` 差值处理时钟回绕
- 服务每轮有界消费队列，默认快照年龄上限 10 ms、相邻 gyro 间隔上限 20 ms
- 消费期间到达的 DMA 样本可有界领先入口 `now_us`，控制检查年龄时须重新读取当前时钟
- 队列空返回 `NOT_READY`，保留仍新鲜的快照与静止累计，持续无数据超过门槛才失效

## 姿态与编码器的关系

本项目使用 `IMU_ATTITUDE_MAHONY_6AXIS`，由三轴陀螺积分四元数，并用可信加速度的重力误差修正三个轴

`gyro_corrected` 仅扣除零偏与温漂，不混入 Mahony 比例反馈，默认 `mahony_ki` 和 `mahony_ki_z` 均为 0，姿态服务初始化时检查这一约束

编码器不输入 Mahony 更新，在 `domain/gimbal_control` 中用于关节映射、机械限位、Z-X-Y Jacobian、目标生成和速度前馈，反馈速度还参与已校准后的静止学习许可

六轴 IMU 没有绝对航向观测，重力反馈不能消除世界系 yaw 的长期漂移

## 零偏与静止学习

```text
effective_bias = startup_bias + temp_coeff * (T - startup_T) + residual_bias
gyro_corrected = gyro_raw - effective_bias
```

准备期先确认 0.5 s 持续静止，再累计 2000 个真实 gyro 样本，约 1 kHz 采集时合计约 2.5 s

启动标定由 IMU 的 gyro 均值、加速度模长和持续统计窗口判断，不等待电机反馈，标定后更新残余零偏还须无故障、三轴反馈在 200 ms 内且低速

学习许可不等于静止，整机仍须放稳，RUN 撤销学习许可并冻结残余零偏，温度补偿继续生效

校准完成的同一帧，用新得到的有效零偏平移已有 gyro 均值，保留均值历史、静止时长和加速度窗口

```text
effective_offset = gyro_raw - gyro_corrected
gyro_mean_unbiased = gyro_mean_raw - effective_offset
```

运动、方差超限、时间断续、数据过期或采集错误仍会清除静止候选，已完成的启动校准保持，控制可用性另由数据有效性判断

温漂斜率沿用参考参数，具体传感器仍须用升温日志核对

## 状态接口

| 字段或接口 | 含义 |
|---|---|
| `attitude.get_snapshot(&snapshot)` | 复制最近快照，返回 true 不等于可用于控制 |
| `snapshot.valid` / `state.ready` | 最近姿态已校准、更新成功且原始数据未超龄 |
| `state.calibrated` | 启动标定完成，不代表当前数据新鲜 |
| `calibration_samples / calibration_target` | 已累计与所需真实样本数 |
| `stationary_candidate_s` | 连续静止候选时长 |
| `bias_learning_allowed` | 外部学习许可，不是静止结论 |
| `static_reset_count` | 已有候选或未完成校准进度被清除的累计次数 |
| `last_reset_reason` | 最近历史清除或阻塞原因，恢复后保留 |

空窗口被阻塞时可能只更新历史原因，不增加清除次数，这些字段都不是 MCU 重启计数或当前故障标志

## 驱动复用约束

- `bmi088_instance` 依赖 DRDY、SPI DMA 与完成/错误回调转发
- 提供 `now_us` 可保留微秒时间戳，缺省时回退到 `now_ms() * 1000`
- 同一主循环先调用 `update()` 再复制 `Bmi088Snapshot`，复制不消费队列，ISR/DMA 不写该快照
- DMA 缓冲须满足目标平台内存可达性、对齐与缓存维护要求
- `bmi088_blocking_instance` 保留给独立设备调试，本项目不使用
- `imu_get_angle()` 等驱动内姿态接口保留兼容，本项目控制只使用 attitude 服务输出
- 单独复用 `imu_attitude` 时，先提供持续静止结论，再送入带有效位和时间戳的 `ImuSample`

模式、命令和日志格式见工程根目录 [README](../../../README.md)
