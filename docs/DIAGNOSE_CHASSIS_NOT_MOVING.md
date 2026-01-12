# 底盘不动问题诊断指南

## 🎯 问题现象
在Jetson上运行 `cmd_vel_forwarder.py` 发送命令，但底盘不动。

---

## 📋 诊断步骤

### 步骤1：确认烧录的版本是否正确 ⚠️ **最重要**

**问题**：如果你烧录的是 `infantry.elf`，但你的机器人是 `sentry_swerve`，那么：
- 电机配置不匹配
- 运动学解算不匹配
- 转向电机初始化逻辑不匹配

**检查方法**：

1. **查看构建目录**：
   ```bash
   # 如果你用的是 build_infantry，那可能是错的
   ls build_infantry/NYUSH_Infantry.elf
   
   # 应该用 build_sentry
   ls build_sentry/NYUSH_Infantry.elf
   ```

2. **重新构建 sentry 版本**（如果之前用的是 infantry）：
   ```powershell
   # 在 Windows 上
   cmake --preset Debug -DROBOT_TYPE=sentry_swerve -B build_sentry
   cmake --build build_sentry
   ```

3. **重新烧录**：
   - 使用 STM32CubeProgrammer 烧录 `build_sentry/NYUSH_Infantry.hex` 或 `.elf`

---

### 步骤2：查看STM32日志输出（USB CDC）

STM32会通过USB CDC发送日志，你可以看到：
- 是否收到radar数据
- CRC校验是否通过
- 数据是否超时
- 电机初始化状态

**在Jetson上查看日志**：

```bash
# 方法1：使用 screen（推荐）
sudo apt-get install screen
screen /dev/ttyACM0 115200

# 方法2：使用 minicom
sudo apt-get install minicom
sudo minicom -D /dev/ttyACM0 -b 115200

# 方法3：使用 cat（简单但可能乱码）
cat /dev/ttyACM0
```

**预期看到的日志**：

如果数据正常接收，你应该看到：
```
[DEBUG] RADAR OK: vx=0.500 vy=0.000 wz=0.000 frames=123 errors=0
[DEBUG] RADAR STATUS: buf_used=0/256 frames=123 errors=0 valid=1
```

如果CRC错误：
```
[DEBUG] RADAR CRC ERR: calc=0xAB recv=0xCD
```

如果超时：
```
[DEBUG] RADAR TIMEOUT: no data for 501ms
```

**如果没有看到任何日志**：
- 检查USB连接
- 检查端口权限：`sudo chmod 666 /dev/ttyACM0`
- 检查是否启用了日志：查看 `modules/logger/logger_config.h` 中的 `LOG_ENABLE_DEBUG`

---

### 步骤3：检查Python脚本发送频率

**问题**：如果发送频率太低（< 2Hz），数据可能超过500ms超时，导致fallback到RC模式。

**检查方法**：

1. **查看Python脚本输出**：
   ```bash
   python3 cmd_vel_forwarder.py --port /dev/ttyACM0 --ros2
   ```
   应该看到每收到一个ROS2消息就打印一次。

2. **检查ROS2消息频率**：
   ```bash
   ros2 topic hz /cmd_vel
   ```
   应该 > 10Hz（推荐20-50Hz）。

3. **如果使用one-shot模式**，确保发送频率足够：
   ```bash
   # 默认是20Hz，应该够用
   python3 cmd_vel_forwarder.py --port /dev/ttyACM0 --vx 0.5 --rate 20
   ```

---

### 步骤4：检查电机初始化（sentry_swerve特有）

**问题**：对于 `sentry_swerve`，转向电机（GM6020）需要先对齐到 `initial_angle`。

**检查方法**：

1. **查看初始化日志**：
   在STM32日志中应该看到：
   ```
   [SENTRY] Waiting for steer alignment...
   [SENTRY] Steer motor 0: target=1084.0 current=1085.0 diff=1.0
   [SENTRY] Steer motor 1: target=2434.0 current=2433.0 diff=1.0
   [SENTRY] Steer alignment complete
   ```

2. **如果对齐失败**：
   - 检查 `config/sentry_swerve.h` 中的 `initial_angle` 是否正确
   - 检查CAN总线连接
   - 检查电机ID配置

3. **手动触发对齐**（如果需要）：
   查看 `application/chassis/sentry_controller.c` 中的 `Sentry_WaitForSteerAlignment()` 函数。

---

### 步骤5：检查底盘命令是否启用

**检查方法**：

在STM32日志中查找：
```
[CMD] RADAR,IN:0.500,0.000,0.000,OUT:0.500,0.000,0.000,sp0:10.00,sp1:10.00
```

如果看到这个日志，说明：
- ✅ 数据已接收
- ✅ 数据已解析
- ✅ 已转换为电机速度

如果**没有看到这个日志**，可能：
- 数据超时（>500ms）
- `s_last_radar.valid` 为false
- 控制模式没有切换到RADAR模式

---

### 步骤6：检查电机驱动和CAN通信

**检查方法**：

1. **查看CAN通信日志**（如果启用了）：
   ```
   [CAN] Motor 0x201: speed=1000 rpm, angle=1084
   ```

2. **检查电机反馈**：
   - 如果电机没有反馈，`ChassisController_ComputeCurrents` 可能返回0电流
   - 检查CAN总线连接
   - 检查电机ID配置

3. **检查电机使能**：
   - 确保电机驱动器已上电
   - 确保CAN总线正常通信

---

## 🔧 快速诊断命令

在Jetson上运行以下命令，快速检查：

```bash
# 1. 检查端口是否存在
ls -l /dev/ttyACM*

# 2. 检查端口权限
sudo chmod 666 /dev/ttyACM0

# 3. 查看STM32日志（新终端）
screen /dev/ttyACM0 115200

# 4. 发送测试命令（另一个终端）
python3 cmd_vel_forwarder.py --port /dev/ttyACM0 --vx 0.5 --vy 0.0 --wz 0.0 --rate 20

# 5. 检查ROS2消息频率（如果使用ROS2）
ros2 topic hz /cmd_vel
```

---

## 🐛 常见问题

### Q1: 完全没有日志输出
**可能原因**：
- USB连接问题
- 日志未启用
- 端口权限问题

**解决方法**：
```bash
sudo chmod 666 /dev/ttyACM0
# 检查 logger_config.h 中的 LOG_ENABLE_DEBUG
```

### Q2: 看到CRC错误
**可能原因**：
- 串口波特率不匹配
- 数据损坏
- Python脚本发送格式错误

**解决方法**：
- 检查波特率：确保Python和STM32都是115200
- 检查Python脚本的 `encode_radar_cmd` 函数
- 检查STM32的 `RadarComm_Task` 中的CRC计算

### Q3: 看到超时日志
**可能原因**：
- 发送频率太低
- 串口连接不稳定

**解决方法**：
- 提高发送频率（>10Hz）
- 检查USB线连接
- 检查是否有其他程序占用串口

### Q4: 数据正常但底盘不动
**可能原因**：
- 电机未初始化
- CAN通信问题
- 电机驱动器未使能
- 底盘命令未启用（`s_chassis_cmd.enabled = false`）

**解决方法**：
- 检查电机初始化日志
- 检查CAN总线连接
- 检查电机驱动器状态
- 检查 `ChassisController_ComputeCurrents` 是否被调用

---

## 📝 下一步

如果以上步骤都检查过了，还是不动，请提供：
1. STM32日志输出（从screen/minicom复制）
2. Python脚本输出
3. 你烧录的是哪个版本（infantry还是sentry）
4. 你的机器人类型（infantry_standard还是sentry_swerve）
