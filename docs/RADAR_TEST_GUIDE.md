# Radar通信测试指南

## 📋 测试步骤

### 第一步：编译并烧录STM32代码

1. **编译代码**
   ```bash
   # 在项目根目录
   cd build_infantry  # 或 build_sentry，根据你的机器人类型
   cmake ..
   cmake --build .
   ```

2. **烧录到STM32**
   - 使用ST-Link或J-Link烧录生成的 `.hex` 或 `.bin` 文件
   - 确保USB线连接到STM32的USB接口

### 第二步：连接硬件

1. **USB连接**
   - 用USB线连接STM32到电脑（Jetson或PC）
   - 等待系统识别USB CDC设备

2. **查看端口号**
   ```bash
   # Linux (Jetson/Ubuntu)
   ls /dev/ttyACM*  # 或 /dev/ttyUSB*
   
   # Windows
   # 在设备管理器中查看"端口(COM和LPT)"，找到STM32 Virtual COM Port
   # 通常是 COM3, COM4 等
   ```

3. **赋予权限（Linux）**
   ```bash
   sudo chmod 666 /dev/ttyACM0  # 替换为你的实际端口
   ```

### 第三步：运行Python脚本

**方式A：ROS2模式（推荐）**
```bash
# 终端1: 启动Python转发脚本
python3 script/cmd_vel_forwarder.py --port /dev/ttyACM0 --ros2

# 终端2: 发送ROS2命令测试
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

**方式B：直接发送模式**
```bash
# 发送固定速度命令（vx=0.5 m/s）
python3 script/cmd_vel_forwarder.py --port /dev/ttyACM0 --vx 0.5 --vy 0.0 --wz 0.0
```

### 第四步：查看STM32输出（确认数据接收）

Python脚本会自动显示STM32的输出（通过USB CDC）。你应该能看到类似这样的输出：

#### ✅ 成功接收数据的标志：

**1. 收到有效帧（每500ms输出一次）**
```
STM32> [DEBUG][INFO] RADAR OK: vx=0.500 vy=0.000 wz=0.000 frames=10 errors=0
```

**2. 状态报告（每2秒输出一次）**
```
STM32> [DEBUG][INFO] RADAR STATUS: buf_used=0/128 frames=50 errors=0 valid=1
```

**3. 命令控制器日志（如果启用）**
```
STM32> [CMD][CSV] RADAR,IN:0.500,0.000,0.000,OUT:0.500,0.000,0.000,sp0:10.00,sp1:10.00
```

#### ❌ 如果看到这些，说明有问题：

**CRC错误**
```
STM32> [DEBUG][DEBUG] RADAR CRC ERR: calc=0xAB recv=0xCD
```
- **原因**：数据传输损坏
- **解决**：检查USB线连接，尝试降低发送频率

**超时**
```
STM32> [DEBUG][DEBUG] RADAR TIMEOUT: no data for 250ms
```
- **原因**：超过200ms没有收到新数据
- **解决**：检查Python脚本是否在运行，检查端口是否正确

**缓冲区满**
```
STM32> [DEBUG][INFO] RADAR STATUS: buf_used=128/128 frames=0 errors=0 valid=0
```
- **原因**：数据接收太快，处理不过来
- **解决**：降低Python发送频率（`--rate 10`）

### 第五步：验证底盘响应

**⚠️ 重要：首次测试请将底盘架空！**

1. **观察轮子**
   - 如果收到数据，轮子应该开始转动
   - 速度应该与发送的 `vx`, `vy`, `wz` 值对应

2. **检查控制模式**
   - STM32会优先使用radar数据（如果有效）
   - 如果radar数据无效，会回退到遥控器控制

## 🔍 调试技巧

### 1. 使用虚拟串口测试（Linux，无需硬件）

```bash
# 终端1: 创建虚拟串口对
socat -d -d pty,raw,echo=0 pty,raw,echo=0
# 输出: /dev/pts/2 和 /dev/pts/3

# 终端2: Python发送
python3 script/cmd_vel_forwarder.py --port /dev/pts/2 --vx 0.5

# 终端3: 查看原始Hex数据
hexdump -C < /dev/pts/3
# 应该看到: a5 5a 00 00 00 3f 00 00 00 00 ...
```

### 2. 检查Python发送的数据

```bash
# 使用hexdump查看Python发送的原始数据
python3 script/cmd_vel_forwarder.py --port /dev/ttyACM0 --vx 0.5 &
# 然后用另一个工具监听端口
```

### 3. 增加调试输出

如果看不到输出，检查：
- `modules/logger/logger_config.h` 中 `LOG_ENABLE_DEBUG` 和 `LOG_ENABLE_CMD` 是否为 `1`
- 重新编译并烧录

### 4. 检查USB连接

```bash
# Linux: 检查USB设备
lsusb | grep STM

# 检查dmesg日志
dmesg | tail -20
```

## 📊 预期输出示例

### 正常工作的完整输出：

```
STM32> ========================================
STM32>    RoboMaster Control System Boot
STM32> ========================================
STM32> [DEBUG][INFO] RADAR STATUS: buf_used=0/128 frames=0 errors=0 valid=0
STM32> [DEBUG][INFO] RADAR OK: vx=0.200 vy=0.000 wz=0.000 frames=1 errors=0
STM32> [DEBUG][INFO] RADAR OK: vx=0.200 vy=0.000 wz=0.000 frames=2 errors=0
STM32> [CMD][CSV] RADAR,IN:0.200,0.000,0.000,OUT:0.200,0.000,0.000,sp0:4.00,sp1:4.00
STM32> [DEBUG][INFO] RADAR STATUS: buf_used=0/128 frames=20 errors=0 valid=1
```

## 🐛 常见问题

### Q1: Python脚本报错 "Permission denied"
**A:** 需要赋予串口权限：
```bash
sudo chmod 666 /dev/ttyACM0
# 或添加到dialout组
sudo usermod -a -G dialout $USER
```

### Q2: 看不到STM32输出
**A:** 
1. 检查USB连接
2. 检查 `logger_config.h` 中的日志开关
3. 尝试用 `minicom` 或 `screen` 直接连接串口：
   ```bash
   screen /dev/ttyACM0 115200
   ```

### Q3: 收到数据但轮子不转
**A:**
1. 检查底盘控制模式（radar数据是否有效）
2. 检查电机是否初始化
3. 检查CAN总线连接
4. 查看 `cmd_controller.c` 中的日志输出

### Q4: CRC错误很多
**A:**
1. USB线质量问题，尝试更换USB线
2. 降低发送频率：`--rate 10`
3. 检查是否有电磁干扰

## ✅ 成功标准

- ✅ Python脚本正常运行，无错误
- ✅ STM32输出显示 "RADAR OK" 消息
- ✅ `frames` 计数持续增加
- ✅ `errors` 计数为0或很少
- ✅ `valid=1` 表示数据有效
- ✅ 底盘轮子按预期转动（如果连接了电机）

## 📝 下一步

一旦确认数据接收正常，你可以：
1. 调整底盘运动学参数（在 `cmd_controller.c` 中）
2. 调整滤波参数（`RADAR_SMOOTH_ALPHA`, `RADAR_MAX_DELTA_V` 等）
3. 集成到ROS2导航栈
4. 添加安全限制（最大速度、加速度限制等）
