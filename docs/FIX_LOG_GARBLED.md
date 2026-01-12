# 解决STM32日志乱码问题

## 🎯 问题现象
使用 `screen` 或 `minicom` 查看STM32日志时，显示乱码。

---

## 🔍 原因分析

USB CDC（虚拟串口）**不需要设置波特率**，但Linux终端工具可能会尝试设置波特率，导致乱码。

STM32发送的数据格式：
- **数据格式**：纯文本（ASCII）
- **行结束符**：`\r\n`（Windows风格）
- **编码**：ASCII（不是UTF-8）

---

## ✅ 解决方法

### 方法1：使用 `screen`（推荐）

**关键**：不要指定波特率，或者使用任意波特率（screen会忽略USB CDC的波特率设置）

```bash
# 方法A：不指定波特率（最简单）
screen /dev/ttyACM0

# 方法B：指定任意波特率（screen会忽略，但可以避免警告）
screen /dev/ttyACM0 115200

# 退出screen：按 Ctrl+A，然后按 K，再按 Y 确认
```

**如果还是乱码，尝试设置终端编码**：
```bash
# 设置终端为ASCII编码
export LANG=C
screen /dev/ttyACM0
```

---

### 方法2：使用 `minicom`

```bash
# 安装minicom（如果没有）
sudo apt-get install minicom

# 配置minicom（首次使用）
sudo minicom -s

# 在配置菜单中：
# 1. 选择 "Serial port setup"
# 2. 设置 Serial Device: /dev/ttyACM0
# 3. 设置 Bps/Par/Bits: 115200 8N1
# 4. 保存配置（Save setup as dfl）
# 5. 退出配置（Exit）

# 使用minicom
sudo minicom -D /dev/ttyACM0 -b 115200
```

**如果还是乱码**：
```bash
# 设置编码为ASCII
export LANG=C
sudo minicom -D /dev/ttyACM0 -b 115200
```

---

### 方法3：使用 `cat`（最简单，但可能不实时）

```bash
# 直接查看原始数据（可能显示不完整）
cat /dev/ttyACM0

# 使用hexdump查看原始字节（调试用）
hexdump -C /dev/ttyACM0
```

---

### 方法4：使用 `picocom`（推荐用于调试）

```bash
# 安装picocom
sudo apt-get install picocom

# 使用picocom（不设置波特率，让系统自动检测）
picocom /dev/ttyACM0

# 或者明确指定（虽然USB CDC不需要）
picocom /dev/ttyACM0 -b 115200

# 退出：按 Ctrl+A，然后按 X
```

---

### 方法5：使用专用Python脚本（⭐ 推荐，解决乱码最有效）

**项目已包含专用脚本**：`script/view_stm32_logs.py`

这个脚本专门设计来处理STM32日志的乱码问题：
- ✅ 自动过滤二进制数据（radar/vision帧）
- ✅ 正确处理ASCII文本
- ✅ 处理编码问题
- ✅ 显示清晰的日志输出

**使用方法**：

```bash
# 基本用法（推荐）
python3 script/view_stm32_logs.py /dev/ttyACM0

# 如果需要查看原始数据（调试用）
python3 script/view_stm32_logs.py /dev/ttyACM0 --raw

# 不过滤二进制数据（不推荐，会显示很多乱码）
python3 script/view_stm32_logs.py /dev/ttyACM0 --no-filter
```

**如果脚本不存在，手动创建**：

```python
#!/usr/bin/env python3
import serial
import sys

port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM0'
ser = serial.Serial(port, timeout=1.0)
print(f"Connected to {port}. Press Ctrl+C to exit.\n")
try:
    while True:
        line = ser.readline()
        if line:
            # 只显示可打印的ASCII字符
            text = ''.join(c if 32 <= ord(c) <= 126 or c in '\r\n\t' else '' 
                          for c in line.decode('ascii', errors='ignore'))
            if text.strip():
                print(text, end='')
except KeyboardInterrupt:
    print("\nExiting...")
finally:
    ser.close()
```

---

## 🔧 常见问题排查

### Q1: 仍然显示乱码

**检查1：确认端口权限**
```bash
# 检查权限
ls -l /dev/ttyACM0

# 如果没有权限，添加当前用户到dialout组
sudo usermod -a -G dialout $USER
# 然后重新登录，或使用
sudo chmod 666 /dev/ttyACM0
```

**检查2：确认端口没有被其他程序占用**
```bash
# 查看是否有其他程序在使用
lsof /dev/ttyACM0

# 如果有，关闭它们
sudo killall screen
sudo killall minicom
```

**检查3：尝试不同的终端工具**
```bash
# 尝试picocom
picocom /dev/ttyACM0

# 或使用Python脚本
python3 view_logs.py /dev/ttyACM0
```

---

### Q2: 日志显示不完整或断断续续

**可能原因**：
- USB连接不稳定
- 缓冲区溢出
- 发送速度太快

**解决方法**：
```bash
# 使用Python脚本，设置更大的缓冲区
python3 view_logs.py /dev/ttyACM0
```

---

### Q3: 完全没有输出

**检查1：确认STM32已连接**
```bash
# 查看USB设备
lsusb | grep STM

# 查看串口设备
ls -l /dev/ttyACM*
```

**检查2：确认日志已启用**
- 检查 `modules/logger/logger_config.h` 中的 `LOG_ENABLE_DEBUG` 等宏
- 确保相关日志标签已启用

**检查3：尝试重新连接**
```bash
# 重新插拔USB线
# 或使用udev规则自动设置权限
```

---

## 📝 推荐工作流程（按优先级）

### ⭐ 方法1：使用项目自带的Python脚本（最推荐）

```bash
# 1. 设置权限
sudo chmod 666 /dev/ttyACM0

# 2. 使用专用脚本（自动处理乱码）
python3 script/view_stm32_logs.py /dev/ttyACM0
```

**优点**：
- ✅ 自动过滤二进制数据
- ✅ 正确处理编码
- ✅ 显示清晰的日志
- ✅ 不需要额外配置

### 方法2：使用screen（如果Python脚本不可用）

```bash
# 设置权限
sudo chmod 666 /dev/ttyACM0

# 设置ASCII编码
export LANG=C

# 使用screen
screen /dev/ttyACM0
```

### 方法3：使用picocom（备选）

```bash
sudo apt-get install picocom
picocom /dev/ttyACM0
```

---

## 🎯 预期看到的日志格式

如果配置正确，你应该看到类似这样的日志：

```
[SYS][INFO] ========================================
[SYS][INFO]    RoboMaster Control System Boot
[SYS][INFO] ========================================
[DEBUG][INFO] RADAR OK: vx=0.500 vy=0.000 wz=0.000 frames=123 errors=0
[DEBUG][INFO] RADAR STATUS: buf_used=0/256 frames=123 errors=0 valid=1
```

或者CSV格式：
```
DEBUG,12345,RADAR OK: vx=0.500 vy=0.000 wz=0.000 frames=123 errors=0
CMD,12350,RADAR,IN:0.500,0.000,0.000,OUT:0.500,0.000,0.000,sp0:10.00,sp1:10.00
```

---

## 💡 提示

- **USB CDC不需要波特率**：虽然可以设置，但实际传输速度由USB决定
- **使用ASCII编码**：STM32发送的是ASCII文本，不是UTF-8
- **行结束符是`\r\n`**：Windows风格，Linux终端通常能正确处理
- **如果所有方法都失败**：检查USB线、驱动、或STM32代码中的日志发送逻辑
