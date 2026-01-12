# 更新Jetson上的cmd_vel_forwarder.py

## 🎯 需要更新的原因

你Jetson上的版本缺少以下改进：
1. ❌ 没有二进制数据过滤（会显示很多乱码）
2. ❌ 没有线程锁（可能有多线程安全问题）
3. ❌ 没有重连机制（连接断开后无法自动恢复）

---

## ✅ 更新方法

### 方法1：直接替换文件（推荐）

#### 步骤1：在Windows上准备更新后的文件

文件已创建：`script/cmd_vel_forwarder_updated.py`

#### 步骤2：传输到Jetson

```powershell
# 在Windows PowerShell中
cd C:\Users\zhuya\Desktop\robomaster\robo-electro-control\robomaster-control

# 传输更新后的文件
scp -P 7913 script/cmd_vel_forwarder_updated.py nyu@42.192.208.124:/home/nyu/robomaster-control/script/cmd_vel_forwarder.py
```

**说明**：直接覆盖原文件。

#### 步骤3：在Jetson上验证

```bash
# SSH登录
ssh -p 7913 nyu@42.192.208.124

# 检查文件
cd ~/robomaster-control/script
head -20 cmd_vel_forwarder.py

# 测试
python3 cmd_vel_forwarder.py --port /dev/ttyACM0 --vx 0.5 --rate 20
```

---

### 方法2：手动修改（如果不想传输）

在Jetson上编辑文件：

```bash
# SSH登录
ssh -p 7913 nyu@42.192.208.124

# 编辑文件
nano ~/robomaster-control/script/cmd_vel_forwarder.py
```

#### 需要修改的部分：

**1. 在 `__init__` 方法中添加线程锁和重连变量**：

找到：
```python
def __init__(self, port: str, baud: int = 115200, timeout=1.0):
    self.port = port
    self.baud = baud
    self.timeout = timeout
    self.ser = None
```

改为：
```python
def __init__(self, port: str, baud: int = 115200, timeout=1.0):
    self.port = port
    self.baud = baud
    self.timeout = timeout
    self.ser = None
    self.lock = threading.Lock()  # 线程锁
    self.last_reconnect_attempt = 0
    self.reconnect_interval = 2.0  # 2秒重连冷却
```

**2. 修改 `open` 方法**：

找到：
```python
def open(self):
    self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
    time.sleep(0.2)
    # Start background reader to print incoming STM32 logs
    self._reader_run = True
    self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
    self._reader_thread.start()
```

改为：
```python
def open(self):
    with self.lock:
        if self.ser and self.ser.is_open:
            return
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
            time.sleep(0.2)
            # 启动读取线程（避免重复启动）
            if not getattr(self, '_reader_run', False):
                self._reader_run = True
                self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
                self._reader_thread.start()
            print(f"Serial port {self.port} opened.")
        except Exception as e:
            print(f"Error opening serial port: {e}")
            self.ser = None
```

**3. 修改 `send` 方法**：

找到：
```python
def send(self, vx: float, vy: float, wz: float):
    frame = encode_radar_cmd(vx, vy, wz)
    if not self.ser or not self.ser.is_open:
        self.open()
    self.ser.write(frame)
    # Note: flush() removed - let OS handle buffering for better performance
```

改为：
```python
def send(self, vx: float, vy: float, wz: float):
    frame = encode_radar_cmd(vx, vy, wz)
    
    # 检查是否连接，带有冷却机制
    if not self.ser or not self.ser.is_open:
        now = time.time()
        if now - self.last_reconnect_attempt > self.reconnect_interval:
            self.last_reconnect_attempt = now
            self.open()
        return  # 如果还在冷却期或打开失败，直接丢弃这帧数据，不要阻塞

    try:
        with self.lock:
            self.ser.write(frame)
            # Note: flush() removed - let OS handle buffering for better performance
    except Exception as e:
        print(f"Serial write error: {e}")
        self.close()  # 出错时关闭，等待下次重连
```

**4. 修改 `close` 方法**：

找到：
```python
def close(self):
    if self.ser and self.ser.is_open:
        self._reader_run = False
        try:
            self.ser.close()
        except Exception:
            pass
```

改为：
```python
def close(self):
    with self.lock:
        self._reader_run = False
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
```

**5. 修改 `_reader_loop` 方法（最重要，过滤二进制数据）**：

找到：
```python
def _reader_loop(self):
    while self._reader_run:
        ser_obj = self.ser
        if ser_obj and ser_obj.is_open:
            try:
                # 使用非阻塞方式或带超时的读取
                if ser_obj.in_waiting > 0:
                    line = ser_obj.readline()
                    if line:
                        print(f'STM32> {line}')
                else:
                    time.sleep(0.01) # 避免空转占用 CPU
            except Exception:
                time.sleep(0.1)
        else:
            time.sleep(0.5)
```

改为：
```python
def _reader_loop(self):
    # Read lines from STM32 and print to stdout
    while getattr(self, '_reader_run', False) and self.ser and self.ser.is_open:
        try:
            line = self.ser.readline()
            if line:
                # Filter out binary radar frames (start with 0xA5 0x5A)
                if len(line) >= 2 and line[0] == 0xA5 and line[1] == 0x5A:
                    continue  # Skip radar frames
                
                # Check if it's mostly printable text
                printable_count = sum(1 for b in line if (32 <= b <= 126) or b in (9, 10, 13))
                if len(line) > 0 and printable_count / len(line) > 0.8:
                    # It's text, decode and print
                    try:
                        s = line.decode('ascii', errors='ignore').rstrip('\r\n')
                        # Only print if it's not empty and contains actual text
                        if s.strip():
                            print('STM32>', s)
                    except Exception:
                        pass
                # Skip binary data (don't print hex dumps to reduce noise)
        except Exception:
            # Short sleep to avoid busy loop on errors
            time.sleep(0.05)
```

---

## 📝 快速更新命令（推荐方法1）

```powershell
# 在Windows PowerShell中
cd C:\Users\zhuya\Desktop\robomaster\robo-electro-control\robomaster-control
scp -P 7913 script/cmd_vel_forwarder_updated.py nyu@42.192.208.124:/home/nyu/robomaster-control/script/cmd_vel_forwarder.py
```

然后在Jetson上测试：

```bash
python3 ~/robomaster-control/script/cmd_vel_forwarder.py --port /dev/ttyACM0 --vx 0.5 --rate 20
```

---

## ✅ 更新后的改进

1. ✅ **自动过滤二进制数据**：不再显示乱码的HEX输出
2. ✅ **线程安全**：使用锁保护串口操作
3. ✅ **自动重连**：连接断开后自动尝试重连
4. ✅ **更清晰的日志**：只显示可读的文本日志

---

## 🎯 验证更新

更新后运行测试，应该看到：

```
Sending frames at 20.0 Hz to /dev/ttyACM0
Sent vx=0.500 vy=0.000 wz=0.000
STM32> [DEBUG][INFO] RADAR OK: vx=0.500 vy=0.000 wz=0.000 frames=1 errors=0
STM32> CMD,12345,RADAR,IN:0.500,0.000,0.000,OUT:0.500,0.000,0.000,sp0:10.00,sp1:10.00
```

**不应该看到**：
- `STM32> HEX: a50e00ae...`（二进制数据）
- 乱码字符
