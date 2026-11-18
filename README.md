# Three-Axis Stabilized Gimbal v1.4

STM32H723 三轴增稳云台，达妙与高擎共用 FDCAN1

## 使用

1. 上电放稳，完成 IMU 标定和电机握手
2. 条件就绪后自动进入 RUN，保持进入时的载荷姿态
3. 安全故障停止后按 PA15 确认，输入恢复后自动继续

正常运行时 PA15 不切换模式

启动标识为 `BOOT gimbal-v1.4-20261118`

| 状态 | RGB |
|---|---|
| 初始化、标定、反馈等待、故障、安全停止 | 红快闪，125 ms 翻转 |
| 其他状态 | 熄灭 |

## 坐标

- IMU 为 X 前、Y 左、Z 上
- 机构顺序为外 Yaw 绕 Z、中 Roll 绕 X、内 Pitch 绕 Y
- 三轴右手正方向，机械中位为 0，比例为 1
- 工作范围为三轴 ±45°
- 内部角度 rad，摘要同时显示 rad 与 deg

## 串口

USART1 使用 921600 baud、8N1，命令以 LF 或 CRLF 结束

| 命令 | 功能 |
|---|---|
| `status` | 输出当前状态 |
| `log on` / `log off` | 开关 1 Hz 摘要 |
| `raw on` / `raw off` | 开关 10 Hz 原始数据，10 秒自动结束 |
| `help` | 列出命令 |

日志整帧输出，首尾帧号一致，空间不足时整帧丢弃并计数

WAIT 每次 RUN 最多额外输出一次，FAULT 保存首次异常和停止时的三轴反馈

## 结构

| 目录 | 职责 |
|---|---|
| `src/app` | 应用入口、PA15、状态灯 |
| `src/service` | 运行流程、姿态服务、日志 |
| `src/service/assemble` | 板级配置、依赖绑定、电机运行顺序 |
| `src/domain` | 运动学与姿态位置外环 |
| `src/device` | BMI088、RGB、电机协议 |
| `src/platform` | STM32 HAL 适配 |
| `src/infra` | 时间与日志基础接口 |

保留现有 Interface、配置/PortOps、Doxygen 和分区格式

姿态见 `src/service/attitude.c`，控制见 `src/domain/gimbal_control.c`，参数见 `src/service/assemble/gimbal_config.c`

## 构建

将同名源码更新到现有 EIDE 工程

本包沿用上传包的文件范围，HAL、CMSIS 和其余 CubeMX 生成文件需保留本地版本

## 验证

RGB、运行状态和灯驱动相关的 3 套既有测试共 33 个独立测试进程通过

验证使用独立测试目录，测试文件和构建产物不加入源码包

本次未做上板验证
