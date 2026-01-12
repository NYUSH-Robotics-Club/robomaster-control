# 解决COM端口错误：信号灯超时时间已到

## 🎯 错误信息
```
Failed to open serial port: could not open port 'COM3': OSError(22, '信号灯超时时间已到', None, 121)
```

## 🔍 原因分析

错误代码 **121** 表示"信号灯超时时间已到"（The semaphore timeout period has expired），通常由以下原因引起：

1. **COM端口被其他程序占用**（最常见）
2. **COM端口不存在或未正确识别**
3. **驱动程序问题**
4. **USB连接不稳定**

---

## ✅ 解决方法

### 方法1：检查并关闭占用端口的程序（最重要）

#### 步骤1：查找占用COM3的程序

```powershell
# 查看占用COM3的进程
netstat -ano | findstr COM3

# 或者使用PowerShell命令
Get-Process | Where-Object {$_.Path -like "*COM*"}
```

#### 步骤2：关闭可能占用端口的程序

**常见占用端口的程序**：
- PuTTY
- Tera Term
- Arduino IDE
- STM32CubeIDE
- STM32CubeProgrammer
- 其他串口调试工具
- **你之前打开的日志查看脚本**

**解决方法**：
```powershell
# 关闭所有Python进程（如果日志查看脚本还在运行）
taskkill /F /IM python.exe

# 或者只关闭特定进程（替换PID为实际进程ID）
taskkill /PID <进程ID> /F
```

#### 步骤3：重新尝试

关闭所有可能占用端口的程序后，重新运行：
```powershell
python script/cmd_vel_forwarder.py --port COM3 --vx 0.5 --rate 20
```

---

### 方法2：确认COM端口号是否正确

#### 步骤1：检查设备管理器

1. 按 `Win + X` → **设备管理器**
2. 展开 **"端口(COM和LPT)"**
3. 确认STM32设备显示的COM端口号
4. 如果看到黄色感叹号，可能需要更新驱动

#### 步骤2：列出所有可用COM端口

创建一个简单的Python脚本来列出所有COM端口：

```powershell
python -c "import serial.tools.list_ports; [print(f'{p.device} - {p.description}') for p in serial.tools.list_ports.comports()]"
```

或者创建一个脚本文件 `list_ports.py`：

```python
import serial.tools.list_ports

print("Available COM ports:")
for port in serial.tools.list_ports.comports():
    print(f"  {port.device} - {port.description}")
```

运行：
```powershell
python list_ports.py
```

#### 步骤3：使用正确的COM端口号

如果实际端口不是COM3，使用正确的端口号：
```powershell
python script/cmd_vel_forwarder.py --port COM4 --vx 0.5 --rate 20
```

---

### 方法3：重新插拔USB并重新识别

1. **断开USB连接**
2. **等待5秒**
3. **重新连接USB**
4. **等待Windows识别设备**（设备管理器中的端口号可能会变化）
5. **使用新的COM端口号**

---

### 方法4：修改Python脚本增加重试和更好的错误处理

如果问题持续，可以修改脚本增加重试机制。但首先尝试上面的方法。

---

## 🔧 完整排查流程

### 步骤1：关闭所有可能占用端口的程序

```powershell
# 关闭所有Python进程
taskkill /F /IM python.exe

# 关闭PuTTY（如果安装了）
taskkill /F /IM putty.exe

# 关闭Tera Term（如果安装了）
taskkill /F /IM ttermpro.exe
```

### 步骤2：确认COM端口

```powershell
# 列出所有COM端口
python -c "import serial.tools.list_ports; [print(f'{p.device} - {p.description}') for p in serial.tools.list_ports.comports()]"
```

### 步骤3：检查设备管理器

- 打开设备管理器
- 查看"端口(COM和LPT)"
- 确认STM32设备状态正常（没有黄色感叹号）

### 步骤4：重新连接USB

- 拔掉USB线
- 等待5秒
- 重新插入
- 等待识别完成

### 步骤5：重新尝试

```powershell
# 使用正确的COM端口号
python script/cmd_vel_forwarder.py --port COM3 --vx 0.5 --rate 20
```

---

## 💡 预防措施

### 1. 使用两个终端窗口时

- **窗口1**：查看日志 `python script/view_stm32_logs_windows.py COM3`
- **窗口2**：发送命令 `python script/cmd_vel_forwarder.py --port COM3 ...`

**重要**：如果要切换，先按 `Ctrl+C` 关闭窗口1，再在窗口2中运行命令。

### 2. 使用资源管理器检查端口占用

```powershell
# 查看所有串口资源
Get-PnpDevice -Class Ports | Format-Table FriendlyName, Status, InstanceId
```

---

## 🐛 如果所有方法都失败

### 检查驱动程序

1. 在设备管理器中找到STM32设备
2. 右键 → **更新驱动程序**
3. 选择 **"浏览我的电脑以查找驱动程序"**
4. 或从ST官网下载STM32虚拟COM端口驱动

### 尝试不同的USB端口

- 有些USB端口可能有问题
- 尝试使用USB 2.0端口（而不是USB 3.0）
- 避免使用USB集线器，直接连接到电脑

### 重启电脑

如果以上方法都不行，重启电脑可以清除所有端口占用。

---

## 📝 快速检查清单

- [ ] 关闭所有可能占用端口的程序（Python、PuTTY、Arduino IDE等）
- [ ] 确认COM端口号正确（使用 `list_ports.py` 或设备管理器）
- [ ] 检查设备管理器，确认设备状态正常
- [ ] 重新插拔USB线
- [ ] 使用正确的COM端口号重新尝试

---

## 🎯 最可能的原因

根据错误信息，**最可能的原因是COM端口被其他程序占用**，特别是：
- 你之前打开的日志查看脚本还在运行
- 其他串口调试工具正在使用该端口

**立即尝试**：
```powershell
# 关闭所有Python进程
taskkill /F /IM python.exe

# 然后重新运行
python script/cmd_vel_forwarder.py --port COM3 --vx 0.5 --rate 20
```
