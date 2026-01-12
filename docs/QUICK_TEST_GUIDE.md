# 快速测试指南 - 发送第一个指令

## 🎯 目标
发送一个速度指令给STM32，验证通信链路是否正常。

---

## 📋 步骤1：找到USB端口

### Windows系统

1. **连接USB线**
   - 将USB线连接到STM32的USB接口（不是ST-Link的USB）
   - 等待Windows识别设备

2. **查看端口号**
   - 按 `Win + X` → 选择 **"设备管理器"**
   - 展开 **"端口(COM和LPT)"**
   - 找到 **"STM32 Virtual COM Port"** 或类似设备
   - 记下COM端口号，例如：**COM3** 或 **COM4**

   ```
   端口(COM和LPT)
   └── STM32 Virtual COM Port (COM3)  ← 这就是你要的端口号
   ```

3. **如果没有看到设备**
   - 检查USB线是否连接正确
   - 尝试不同的USB端口
   - 检查STM32是否上电
   - 可能需要安装STM32虚拟COM端口驱动

---

## 📋 步骤2：安装Python依赖（如果还没安装）

```powershell
# 检查是否已安装pyserial
python -m pip list | findstr pyserial

# 如果没有，安装它
python -m pip install pyserial
```

---

## 📋 步骤3：发送测试指令

### 方法A：发送单次指令（推荐首次测试）

打开PowerShell，运行：

```powershell
# 替换COM3为你的实际端口号
python script/cmd_vel_forwarder.py --port COM3 --vx 0.2 --vy 0.0 --wz 0.0
```

**参数说明**：
- `--port COM3`：你的USB端口号
- `--vx 0.2`：X方向速度 0.2 m/s（前进）
- `--vy 0.0`：Y方向速度 0.0 m/s（不侧移）
- `--wz 0.0`：角速度 0.0 rad/s（不旋转）

**预期输出**：
```
Opening serial port COM3...
Sending one-shot command: vx=0.200 vy=0.000 wz=0.000
Frame sent: 15 bytes
STM32> [INFO][SYS] ========================================
STM32> [INFO][SYS]    RoboMaster Control System Boot
STM32> [INFO][SYS] ========================================
STM32> [DEBUG][INFO] RADAR OK: vx=0.200 vy=0.000 wz=0.000 frames=1 errors=0
```

### 方法B：持续发送指令（测试连续控制）

```powershell
# 持续发送指令，每100ms一次（10Hz）
python script/cmd_vel_forwarder.py --port COM3 --vx 0.2 --vy 0.0 --wz 0.0 --rate 10
```

**参数说明**：
- `--rate 10`：发送频率10Hz（每100ms一次）

### 方法C：ROS2模式（如果使用ROS2）

```powershell
# 启动Python脚本，订阅ROS2话题
python script/cmd_vel_forwarder.py --port COM3 --ros2

# 在另一个终端发送ROS2命令
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

---

## 📋 步骤4：观察结果

### ✅ 成功标志

1. **Python脚本输出**：
   ```
   Frame sent: 15 bytes
   ```

2. **STM32响应**（如果启用了日志）：
   ```
   STM32> [DEBUG][INFO] RADAR OK: vx=0.200 vy=0.000 wz=0.000 frames=1 errors=0
   STM32> [DEBUG][INFO] RADAR STATUS: buf_used=0/128 frames=1 errors=0 valid=1
   ```

3. **底盘响应**（如果电机已连接）：
   - 轮子开始转动
   - 前进速度约为 0.2 m/s
   - 如果发送 `wz=0.5`，底盘会旋转

### ❌ 问题排查

**没有输出**：
- 检查端口号是否正确
- 检查USB线是否连接
- 尝试不同的USB端口
- 检查STM32是否上电

**端口被占用**：
```
SerialException: [Error 5] Access is denied
```
- 关闭其他可能占用端口的程序（如串口调试工具）
- 重新插拔USB线

**没有STM32响应**：
- 检查日志是否启用（`logger_config.h`）
- 检查USB CDC是否初始化成功
- 查看STM32的LED是否闪烁（如果有）

---

## 🧪 测试用例

### 测试1：前进
```powershell
python script/cmd_vel_forwarder.py --port COM3 --vx 0.3 --vy 0.0 --wz 0.0
```
**预期**：底盘前进

### 测试2：后退
```powershell
python script/cmd_vel_forwarder.py --port COM3 --vx -0.3 --vy 0.0 --wz 0.0
```
**预期**：底盘后退

### 测试3：侧移
```powershell
python script/cmd_vel_forwarder.py --port COM3 --vx 0.0 --vy 0.3 --wz 0.0
```
**预期**：底盘向左或右侧移（取决于坐标系）

### 测试4：旋转
```powershell
python script/cmd_vel_forwarder.py --port COM3 --vx 0.0 --vy 0.0 --wz 0.5
```
**预期**：底盘原地旋转

### 测试5：组合运动
```powershell
python script/cmd_vel_forwarder.py --port COM3 --vx 0.2 --vy 0.1 --wz 0.3
```
**预期**：底盘同时前进、侧移和旋转

### 测试6：停止
```powershell
python script/cmd_vel_forwarder.py --port COM3 --vx 0.0 --vy 0.0 --wz 0.0
```
**预期**：底盘停止

---

## 🔍 查看详细日志

如果你想看到更多调试信息，可以：

1. **启用更多日志**：
   - 编辑 `modules/logger/logger_config.h`
   - 设置 `LOG_ENABLE_DEBUG 1`
   - 设置 `LOG_ENABLE_CMD 1`
   - 重新编译并烧录

2. **查看Python发送的原始数据**：
   ```python
   # 在cmd_vel_forwarder.py中添加打印
   print(f"Frame hex: {frame.hex()}")
   ```

---

## 📊 完整测试流程

```powershell
# 1. 找到端口号（设备管理器）
#    假设是 COM3

# 2. 发送测试指令
python script/cmd_vel_forwarder.py --port COM3 --vx 0.2 --vy 0.0 --wz 0.0

# 3. 观察输出
#    - Python: "Frame sent: 15 bytes"
#    - STM32: "RADAR OK: vx=0.200..."

# 4. 观察底盘（如果电机已连接）
#    - 轮子应该转动

# 5. 停止
python script/cmd_vel_forwarder.py --port COM3 --vx 0.0 --vy 0.0 --wz 0.0
```

---

## ⚠️ 安全提示

1. **首次测试请将底盘架空**，避免意外移动
2. **从小速度开始**（0.1-0.2 m/s）
3. **准备好停止命令**（vx=0, vy=0, wz=0）
4. **如果失控，立即断开USB或断电**

---

## 🎉 成功！

如果看到：
- ✅ Python显示 "Frame sent"
- ✅ STM32显示 "RADAR OK"
- ✅ 底盘响应速度命令

说明整个链路已经打通！🎊

接下来可以：
- 集成到ROS2系统
- 调整速度限制
- 添加安全保护
- 优化控制算法
