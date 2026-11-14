# 交接说明：1 kHz IMU 串口流 + Mid-360 硬同步

这份文档写给在机载电脑上接手这项工作的人（或另一个 Claude 会话）。它记录了目标、已经做出的决定及其理由、当前代码状态，以及接下来要在硬件上做的事。之前的讨论是在另一台电脑上进行的，那边的聊天记录这里看不到，需要的信息都在这份文档里。

写于 2026-11-14。分支：`HighRateIMU1000Hz`。

## 目标

跳跃机器人要在机载电脑上跑 FAST-LIO2，需要：

1. 从飞控稳定读到 1 kHz 的 IMU 数据，不丢样本。
2. IMU 时间戳和 Livox Mid-360 的点云时间戳硬件同步。

## 硬件

| 部件 | 说明 |
|---|---|
| 飞控 | Crazyflie Bolt，IMU 是 BMI088，量程 accel ±24 g、gyro ±2000°/s |
| 雷达 | Livox Mid-360，带一分三线（有功能线） |
| 机载电脑 | NUC 迷你主机，Intel N305 级别 CPU，16 GB 内存，双 i226-V 网口 |
| 串口模块 | CH340G USB 转 TTL，跳线拨到 3V3 |
| 备用电脑 | Jetson Orin Nano Super 8GB，目前不用 |
| 测量工具 | 只有普通万用表，没有示波器 |

机器人的运动指标决定了方案：支撑相加速度到 20 g，空翻角速度到 1000°/s。

## 已经定下来的决定和理由

- **用飞控的 IMU，不用 Mid-360 自带的 IMU。** 用自带 IMU 跑 FAST-LIO 直接发散，原因是 20 g 的冲击超出它的能力。
- **不用 Crazyflie 的 log 框架。** 单个 log block 上限 26 字节，经典协议最高 100 Hz，时间戳是毫秒 tick，和 IMU 采样不对齐。
- **不用原来的 USB app channel 方案。** 它每个 USB 传输只带一个包，和 log/param 共用 CRTP 队列，主机卡顿超过约 200 ms 就丢样本，而且 micro-USB 插头在落地冲击下可能瞬断。
- **必须硬同步，软同步不够。** 1000°/s 时 1 ms 的时间误差是 1°，10 m 处约 17 cm。
- **飞控当时钟源，雷达锁到飞控。** 飞控输出 PPS 和 GPRMC 给雷达，这样点云和 IMU 用同一个晶振，和机载电脑的时钟无关。PTP 只能让雷达对齐电脑，解决不了 IMU 对齐电脑的问题，所以没有采用。
- **GPRMC 走串口，不走网络。** Livox SDK2 有 `SetLivoxLidarRmcSyncTime` 可以用网络发，但 Livox Wiki 标为不推荐，且要改 `livox_ros_driver2`。
- **发低通滤波之前的数据。** 固件里 accel 有 30 Hz 低通、gyro 有 80 Hz 低通，支撑相只有几十毫秒，滤波后的冲击是错的。
- **保留 NUC，不换 Jetson。** FAST-LIO2 只用 CPU，NUC 更强。只有在 USB 串口模块受冲击掉线且固定不住时才值得换，因为 Jetson 有原生 UART。

## 接线

所有信号都是飞控的 3.0 V 推挽输出。

| 飞控引脚 | 信号 | 接到 |
|---|---|---|
| TX2（PA2） | IMU 帧，1 Mbaud 8N1 | CH340G 的 RXD |
| RX2（PA3） | PPS，1 Hz，上升沿有效，高电平 100 ms | Mid-360 的 8 脚，紫白线 |
| TX1（PC10） | GPRMC，9600 baud 8N1 | Mid-360 的 10 脚，灰白线 |
| GND | 地 | CH340G 的 GND 和 Mid-360 功能线的黑线 |

注意事项：

- CH340G 的 5V/3V3 电源脚不要接飞控。
- Mid-360 的 11 脚（灰线）和 12 脚（紫线）是雷达的输出，悬空并包好。紫白和紫、灰白和灰颜色接近，接之前用万用表对针脚。
- PPS 直接驱动，不要串大电阻（Livox 建议上升沿斜率大于 1 V/µs）。

Mid-360 的同步要求（手册 v1.2 和 Livox Wiki）：PPS 周期 900–1100 ms，高电平大于 1 µs（推荐 10–200 ms），GPRMC 在上升沿之后 0–900 ms 内开始发送（推荐 0–430 ms），只认 GPRMC 或 GNRMC。同步成功后点云时间戳变成 GPRMC 给的时间，数据包里的 `timestamp_type` 为 2，Livox Viewer 2 的设置页显示 Sync Type 为 GPS。

## 固件实现

由 `CONFIG_IMU_UART_STREAM` 控制，已在 `configs/bolt_defconfig` 里打开。

| 文件 | 内容 |
|---|---|
| `src/hal/src/imu_uart_stream.c` | USART2 + DMA 发 IMU 帧，TIM5 输出 PPS，低优先级任务发 GPRMC |
| `src/hal/interface/imu_uart_stream.h` | 接口和接线说明 |
| `src/hal/src/imu_uart_proto.c` | 帧打包、CRC、GPRMC 生成，不依赖硬件 |
| `src/hal/interface/imu_uart_proto.h` | 帧格式定义 |
| `src/hal/src/sensors_bmi088_bmp3xx.c` | 在低通滤波前取数据，调用发送 |

工作方式：

- **PPS**：TIM5 以 1 MHz 计数，每秒回绕一次，通道 4 的 PWM 输出就是 PPS。上升沿由硬件产生，没有中断抖动。TIM5 和 usec 定时器（TIM7）用同一个时钟，所以两者相位固定。
- **时间基准**：开机时的 PPS 沿被定义为 2020-01-01 00:00:00 UTC，之后每个沿加一秒。GPRMC 报的就是这个时间。
- **IMU 时间戳**：数据就绪中断里取的 `usecTimestamp()` 加上一个固定偏移，换算成上述时间基准下的微秒数。主机端不需要再对齐。
- **IMU 帧**：40 字节，小端。同步头 `A5 5A`、版本、标志、16 位序号、64 位时间戳（µs）、accel 三轴 float（G）、gyro 三轴 float（deg/s）、CRC-16/CCITT-FALSE。序号在丢帧时也递增，所以序号的跳变就是丢帧数。
- **不阻塞传感器任务**：上一帧还没发完时直接丢弃并计数。40 字节在 1 Mbaud 下占 0.4 ms，正常情况下不会发生。
- **日志变量**：`imuStream.dropped`（飞控端丢帧数）、`imuStream.pps`（已输出的 PPS 数）、`imuStream.resync`（时间基准被重新校准的次数，应保持为 0）。

占用的资源：UART1、UART2、TIM5、DMA1 stream 6。因此不能和 AI deck、UART1 调试输出、DShot 同时使用，Kconfig 里已经加了依赖限制。

行为变化：

- 打开这个选项后，原来的 app channel IMU 流会被关掉，旧的 ROS 2 `imu_driver` 包收不到数据。
- 时间戳从 2020 年开始计，不是系统时间。
- 不要使用 `usec.reset` 参数，它会让时间基准跳变一次（会自动恢复，`imuStream.resync` 加一）。

## 主机端工具

都在 `tools/imu_uart/`：

| 文件 | 用途 |
|---|---|
| `imu_uart_protocol.py` | 帧解析器，只用标准库 |
| `imu_uart_reader.py` | 命令行工具，每秒打印帧率、丢帧、CRC 错误、采样间隔、最大加速度和角速度 |
| `imu_uart_ros2_node.py` | ROS 2 节点，发布 `sensor_msgs/Imu`（m/s²、rad/s），默认话题 `/cf/imu` |

## 当前状态

已验证：

- Bolt 固件在开发机上编译通过。
- C 的打包、CRC、GPRMC 生成和 Python 解析器对拍一致，覆盖了丢帧、坏帧、乱码、序号回绕和日期进位。

未验证，全部需要在硬件上确认：

- 固件还没有烧录过，串口流、PPS、GPRMC 都没有在真机上跑过。
- PA3 能否输出 PPS。上游舵机驱动里有一句注释说 TX2/RX2 上的 PWM “在 Bolt 1.1 上不工作”，原因不明。
- Mid-360 是否接受这条 GPRMC（位置填的全是 0，日期是 2020 年）。
- `livox_ros_driver2` 发布的消息是否用雷达时间做 header stamp。README 没写，需要读源码。
- CH340G 在 1 Mbaud 下是否丢包。
- ROS 2 节点只做过语法检查，没有运行过。

## 接下来要做的事

按顺序做，每一步通过了再做下一步。

1. **烧录固件。** 固件在开发机上编译（`make bolt_defconfig && make`），产物是 `build/bolt.bin`。`build/` 不在 git 里，要么在开发机上直接烧，要么在机载电脑上装 `arm-none-eabi-gcc` 后重新编译。
2. **只接 CH340G，测串口流。**
   ```
   pip install pyserial
   python3 tools/imu_uart/imu_uart_reader.py /dev/ttyUSB0
   ```
   期望每秒约 1000 帧，`lost` 和 `crc_err` 为 0，`dt_min` 和 `dt_max` 在 1000 µs 附近。如果 `/dev/ttyUSB0` 不出现，Ubuntu 22.04 上通常是 `brltty` 占用了 CH340，卸载它；还要确认用户在 `dialout` 组里。
3. **让机器人落地跳几次**，看 `near_full_scale` 一列有没有接近 24 g 的样本，以及冲击时有没有丢帧。
4. **检查 PPS 和 GPRMC。** 没有示波器时：在 PA3 和 GND 之间接一个 LED 串 1 kΩ 电阻，应该每秒闪一下。GPRMC 可以把 TX1 临时接到 CH340G 的 RXD，用 9600 波特率打开串口，每秒应收到一行 `$GPRMC,...`。
5. **接雷达**，在 Livox Viewer 2 里确认 Sync Type 变成 GPS。
6. **确认 `livox_ros_driver2` 的时间戳来源**，然后运行 ROS 2 节点，把 FAST-LIO2 的 IMU 话题指向 `/cf/imu`，填上雷达到飞控 IMU 的外参，关闭 FAST-LIO2 自带的软同步。
7. **标定固定时间偏移。** BMI088 内部滤波有毫秒量级的固定延迟，硬同步消除不了，要用 FAST-LIO2 的雷达到 IMU 时间偏移参数补偿。

## 出问题时的备选方案

| 现象 | 处理 |
|---|---|
| PA3 没有 PPS | 把 PPS 换到别的引脚。需要一个带定时器通道的空闲引脚，或者改成在高优先级定时器中断里翻转 GPIO |
| CH340G 在 1 Mbaud 下丢包 | 先降到 921600 试；还不行就换 FT232H 或 CP2102N。FT232H 要把 latency timer 改成 1 ms |
| USB 串口模块受冲击掉线 | 短线、扎带、打胶固定；固定不住就换 Jetson，用它排针上的原生 UART |
| 雷达不认 GPRMC | 先确认 PPS 有输出、灰白线和紫白线没接反；再考虑把 GPRMC 里的位置和日期换成更像真实 GPS 的值 |
| 需要时间戳接近系统时间 | 目前纪元是编译期常量 `SYNC_EPOCH_UNIX_S`。可以改成开机后由主机下发一次 UTC 秒数 |
| 接上外部 IMU 后冲击段仍然不稳 | 试 Point-LIO，它把 IMU 当作测量而不是输入，会跳过饱和的轴 |

## 还没解决的已知问题

- BMI088 的 accel 是 1600 Hz 异步采样，gyro 是 1000 Hz 并触发中断，所以 accel 相对时间戳有约 0.6 ms 的不确定度。
- 20 g 离 ±24 g 的量程上限余量不大。
- `configs/bolt_defconfig` 里 `CONFIG_DECK_FORCE="bcServo"` 那一行末尾有一个多余的反引号，是之前就有的，没有动它。舵机驱动仍然被强制加载，实际驱动的是 IO1（PB8），和这里用的引脚不冲突。
