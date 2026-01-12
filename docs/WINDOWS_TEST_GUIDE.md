# Windows系统测试指南

## 🎯 目标
在Windows系统上测试STM32与Python脚本的通信。

---

## 📋 步骤1：找到COM端口

### 方法1：设备管理器（推荐）

1. **打开设备管理器**：
   - 按 `Win + X` → 选择 **"设备管理器"**
   - 或右键"此电脑" → "属性" → "设备管理器"

2. **查找COM端口**：
   - 展开 **"端口(COM和LPT)"**
   - 找到 **"STM32 Virtual COM Port"** 或类似设备
   - 记下COM端口号，例如：**COM3** 或 **COM4**

   ```
   端口(COM和LPT)
   └── STM32 Virtual COM Port (COM3)  ← 这就是你要的端口号
   ```

3. **如果没有看到设备**：
   - 检查USB线是否连接
   - 检查STM32是否上电
   - 尝试不同的USB端口
   - 可能需要安装STM32虚拟COM端口驱动

---

## 📋 步骤2：安装Python依赖

### 检查Python是否已安装

```powershell
python --version
# 或
python3 --version
```

### 安装pyserial

```powershell
python -m pip install pyserial
```

---

## 📋 步骤3：查看STM32日志

### 方法1：使用Python脚本（⭐ 推荐）

**项目已包含Windows版本脚本**：`script/view_stm32_logs_windows.py`

```powershell
# 进入项目目录
cd C:\Users\zhuya\Desktop\robomaster\robo-electro-control\robomaster-control

# 运行日志查看脚本（替换COM3为你的实际端口号）
python script/view_stm32_logs_windows.py COM3
```

**预期输出**：
```
Connected to COM3
Press Ctrl+C to exit

============================================================
[SYS][INFO] ========================================
[SYS][INFO]    RoboMaster Control System Boot
[SYS][INFO] ========================================
[DEBUG][INFO] RADAR OK: vx=0.500 vy=0.000 wz=0.000 frames=123 errors=0
```

---

### 方法2：使用PuTTY（备选）

1. **下载PuTTY**：
   - 访问：https://www.putty.org/
   - 或使用Windows包管理器：`winget install PuTTY.PuTTY`

2. **配置PuTTY**：
   - 打开PuTTY
   - **Connection type**: 选择 **Serial**
   - **Serial line**: 输入COM端口（如 `COM3`）
   - **Speed**: 输入 `115200`（虽然USB CDC不需要，但设置一下）
   - **Data bits**: 8
   - **Stop bits**: 1
   - **Parity**: None
   - **Flow control**: None

3. **连接**：
   - 点击 **Open**
   - 如果看到乱码，尝试：
     - 关闭PuTTY
     - 重新打开，在 **Window → Translation** 中设置：
       - **Character set**: UTF-8 或 **Use font encoding**
       - **Line drawing characters**: 选择 **Use font encoding**

---

### 方法3：使用Tera Term（备选）

1. **下载Tera Term**：
   - 访问：https://ttssh2.osdn.jp/index.html.en

2. **配置Tera Term**：
   - 打开Tera Term
   - 选择 **Serial**，选择COM端口
   - 点击 **Setup → Serial port**：
     - **Baud rate**: 115200
     - **Data**: 8 bit
     - **Parity**: none
     - **Stop**: 1 bit
     - **Flow control**: none

3. **连接**：
   - 点击 **OK** 连接

---

## 📋 步骤4：发送测试命令

### 使用Python脚本发送命令

```powershell
# 进入项目目录
cd C:\Users\zhuya\Desktop\robomaster\robo-electro-control\robomaster-control

# 发送单次测试命令（替换COM3为你的实际端口号）
python script/cmd_vel_forwarder.py --port COM3 --vx 0.5 --vy 0.0 --wz 0.0 --rate 20
```

**预期输出**：
```
Sending frames at 20.0 Hz to COM3
Sent vx=0.500 vy=0.000 wz=0.000
Sent vx=0.500 vy=0.000 wz=0.000
...
```

**同时查看日志**（在另一个PowerShell窗口）：
```powershell
python script/view_stm32_logs_windows.py COM3
```

你应该看到：
```
[DEBUG][INFO] RADAR OK: vx=0.500 vy=0.000 wz=0.000 frames=1 errors=0
[DEBUG][INFO] RADAR OK: vx=0.500 vy=0.000 wz=0.000 frames=2 errors=0
...
```

---

## 📋 步骤5：完整测试流程

### 测试1：检查连接

1. **打开日志查看窗口**（PowerShell 1）：
   ```powershell
   python script/view_stm32_logs_windows.py COM3
   ```

2. **确认看到启动日志**：
   ```
   [SYS][INFO] ========================================
   [SYS][INFO]    RoboMaster Control System Boot
   ```

### 测试2：发送测试命令

1. **打开命令发送窗口**（PowerShell 2）：
   ```powershell
   python script/cmd_vel_forwarder.py --port COM3 --vx 0.5 --rate 20
   ```

2. **在日志窗口应该看到**：
   ```
   [DEBUG][INFO] RADAR OK: vx=0.500 vy=0.000 wz=0.000 frames=1 errors=0
   [DEBUG][INFO] RADAR STATUS: buf_used=0/256 frames=1 errors=0 valid=1
   ```

### 测试3：检查底盘响应

- 如果数据正常接收，底盘应该开始移动
- 如果不动，检查：
  - 电机是否连接
  - CAN总线是否正常
  - 电机初始化是否完成

---

## 🔧 常见问题

### Q1: 找不到COM端口

**解决方法**：
1. 检查USB连接
2. 检查STM32是否上电
3. 在设备管理器中查看是否有"未知设备"
4. 尝试重新插拔USB线
5. 安装STM32虚拟COM端口驱动

### Q2: "Access is denied" 错误

**原因**：端口被其他程序占用

**解决方法**：
```powershell
# 查看占用端口的进程
netstat -ano | findstr COM3

# 结束占用进程（替换PID为实际进程ID）
taskkill /PID <PID> /F
```

### Q3: Python脚本报错 "No module named 'serial'"

**解决方法**：
```powershell
python -m pip install pyserial
```

### Q4: 日志显示乱码

**解决方法**：
1. 使用项目自带的 `view_stm32_logs_windows.py` 脚本（自动过滤二进制数据）
2. 如果使用PuTTY，尝试调整字符编码设置

### Q5: 发送命令但没有响应

**检查清单**：
- [ ] COM端口号是否正确
- [ ] STM32是否已上电
- [ ] USB线是否连接正确
- [ ] 日志中是否看到 `RADAR OK` 消息
- [ ] 发送频率是否足够（>10Hz）

---

## 📝 快速参考

### 常用命令

```powershell
# 查看日志
python script/view_stm32_logs_windows.py COM3

# 发送测试命令（单次）
python script/cmd_vel_forwarder.py --port COM3 --vx 0.5 --rate 20

# 发送测试命令（ROS2模式，如果已配置ROS2）
python script/cmd_vel_forwarder.py --port COM3 --ros2
```

### 端口查找

```powershell
# 使用PowerShell查找COM端口
Get-PnpDevice -Class Ports | Where-Object {$_.FriendlyName -like "*STM32*"}
```

---

## 💡 提示

1. **使用两个PowerShell窗口**：
   - 一个查看日志
   - 一个发送命令

2. **如果端口被占用**：
   - 关闭所有可能使用串口的程序（PuTTY、Tera Term、Arduino IDE等）
   - 重新插拔USB线

3. **测试顺序**：
   - 先确认能看到日志
   - 再发送测试命令
   - 最后检查底盘响应
