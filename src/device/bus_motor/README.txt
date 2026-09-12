ht_motor_final

最终接口文件：
- ht_motor.c
- ht_motor.h

目标：
- 保持 DJI motor driver 风格
- 使用 BusMotorInterface
- 高擎驱动集中在两个文件

接入前请确认：
1. BusMotorFeedback 定义一致
2. BusMotorPortOps send/read 对接 FDCAN-FD
3. 电机型号转换参数来自实际高擎 SDK
