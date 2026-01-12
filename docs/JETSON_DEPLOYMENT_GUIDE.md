# Jetson部署指南 - 从Windows到Jetson

## 🎯 目标
将已验证的Python脚本部署到Jetson上，实现ROS2 → Python → STM32的完整通信链路。

---

## 📋 步骤1：将文件传输到Jetson

### 方法1：使用SCP（推荐）

在Windows PowerShell中：

```powershell
# 进入项目目录
cd C:\Users\zhuya\Desktop\robomaster\robo-electro-control\robomaster-control

# 传输整个script目录到Jetson
scp -r script/ jetson@<Jetson_IP>:/home/jetson/robomaster-control/

# 或者只传输需要的文件
scp script/cmd_vel_forwarder.py jetson@<Jetson_IP>:/home/jetson/robomaster-control/script/
```

**替换 `<Jetson_IP>` 为你的Jetson实际IP地址**，例如：`192.168.1.100`

### 方法2：使用Git（如果Jetson可以访问Git仓库）

在Jetson上：

```bash
cd ~
git clone <your_repo_url> robomaster-control
cd robomaster-control
```

### 方法3：使用U盘或其他方式

直接复制 `script/cmd_vel_forwarder.py` 文件到Jetson。

---

## 📋 步骤2：在Jetson上安装依赖

### 安装Python和pyserial

```bash
# 更新包列表
sudo apt-get update

# 安装Python3（如果还没有）
sudo apt-get install python3 python3-pip

# 安装pyserial
pip3 install pyserial

# 或者使用系统包管理器
sudo apt-get install python3-serial
```

### 安装ROS2（如果需要ROS2集成）

```bash
# 如果还没有安装ROS2，参考ROS2官方文档
# 这里假设已经安装了ROS2

# 检查ROS2是否安装
ros2 --version
```

---

## 📋 步骤3：找到USB端口

### 连接STM32到Jetson

1. 使用USB线连接STM32到Jetson的USB端口
2. 等待系统识别设备

### 查找端口号

```bash
# 方法1：查看所有串口设备
ls -l /dev/ttyACM* /dev/ttyUSB*

# 方法2：使用dmesg查看最新连接的设备
dmesg | tail -20

# 方法3：查看USB设备
lsusb | grep STM

# 方法4：使用Python脚本列出所有端口
python3 -c "import serial.tools.list_ports; [print(f'{p.device} - {p.description}') for p in serial.tools.list_ports.comports()]"
```

**常见端口名**：
- `/dev/ttyACM0` - STM32虚拟COM端口（最常见）
- `/dev/ttyUSB0` - USB转串口设备
- `/dev/ttyACM1` - 如果有多个设备

---

## 📋 步骤4：设置端口权限

### 方法1：临时设置（每次重启后需要重新设置）

```bash
# 设置权限（替换ttyACM0为你的实际端口）
sudo chmod 666 /dev/ttyACM0
```

### 方法2：永久设置（推荐）

```bash
# 将当前用户添加到dialout组
sudo usermod -a -G dialout $USER

# 重新登录或执行以下命令使组权限生效
newgrp dialout

# 验证
groups | grep dialout
```

### 方法3：使用udev规则（最专业）

创建udev规则文件：

```bash
sudo nano /etc/udev/rules.d/99-stm32-cdc.rules
```

添加以下内容：

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE="0666", GROUP="dialout"
```

重新加载udev规则：

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

---

## 📋 步骤5：测试基本通信

### 测试1：发送单次命令

```bash
# 进入项目目录
cd ~/robomaster-control

# 发送测试命令（替换/dev/ttyACM0为你的实际端口）
python3 script/cmd_vel_forwarder.py --port /dev/ttyACM0 --vx 0.5 --vy 0.0 --wz 0.0 --rate 20
```

**预期输出**：
```
Sending frames at 20.0 Hz to /dev/ttyACM0
Sent vx=0.500 vy=0.000 wz=0.000
STM32> [DEBUG][INFO] RADAR OK: vx=0.500 vy=0.000 wz=0.000 frames=1 errors=0
Sent vx=0.500 vy=0.000 wz=0.000
...
```

### 测试2：查看STM32日志（如果需要单独查看）

```bash
# 使用screen（推荐）
sudo apt-get install screen
screen /dev/ttyACM0 115200

# 退出screen：按 Ctrl+A，然后按 K，再按 Y 确认

# 或者使用项目自带的日志查看脚本
python3 script/view_stm32_logs.py /dev/ttyACM0
```

---

## 📋 步骤6：ROS2集成（可选）

### 如果使用ROS2控制

```bash
# 启动ROS2节点
python3 script/cmd_vel_forwarder.py --port /dev/ttyACM0 --ros2

# 在另一个终端发送ROS2命令
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# 或者持续发送
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### 创建ROS2启动文件（可选）

创建 `launch/radar_forwarder.launch.py`：

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='your_package',  # 或者使用executable
            executable='cmd_vel_forwarder.py',
            name='radar_forwarder',
            parameters=[{
                'port': '/dev/ttyACM0',
                'baud': 115200,
                'topic': '/cmd_vel'
            }],
            output='screen'
        )
    ])
```

---

## 📋 步骤7：创建系统服务（可选，用于开机自启动）

### 创建systemd服务文件

```bash
sudo nano /etc/systemd/system/radar-forwarder.service
```

添加以下内容：

```ini
[Unit]
Description=Radar Command Forwarder
After=network.target

[Service]
Type=simple
User=jetson
WorkingDirectory=/home/jetson/robomaster-control
ExecStart=/usr/bin/python3 /home/jetson/robomaster-control/script/cmd_vel_forwarder.py --port /dev/ttyACM0 --ros2
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

启用服务：

```bash
# 重新加载systemd配置
sudo systemctl daemon-reload

# 启用服务（开机自启动）
sudo systemctl enable radar-forwarder.service

# 启动服务
sudo systemctl start radar-forwarder.service

# 查看服务状态
sudo systemctl status radar-forwarder.service

# 查看日志
sudo journalctl -u radar-forwarder.service -f
```

---

## 🔧 常见问题排查

### Q1: 找不到 `/dev/ttyACM0`

**解决方法**：
```bash
# 检查USB连接
lsusb | grep STM

# 检查内核消息
dmesg | tail -20

# 尝试重新插拔USB线
# 检查USB线是否支持数据传输（有些USB线只能充电）
```

### Q2: "Permission denied" 错误

**解决方法**：
```bash
# 临时解决
sudo chmod 666 /dev/ttyACM0

# 永久解决（推荐）
sudo usermod -a -G dialout $USER
newgrp dialout
```

### Q3: "No module named 'serial'"

**解决方法**：
```bash
pip3 install pyserial
# 或
sudo apt-get install python3-serial
```

### Q4: 端口被占用

**解决方法**：
```bash
# 查看占用端口的进程
lsof /dev/ttyACM0

# 关闭占用进程
sudo killall python3
# 或
sudo kill <PID>
```

### Q5: 数据发送但底盘不动

**检查清单**：
- [ ] STM32日志显示 `RADAR OK` 消息
- [ ] 发送频率足够（>10Hz，推荐20Hz）
- [ ] 电机已连接并上电
- [ ] CAN总线正常
- [ ] 遥控器已断开或归中（避免RC模式干扰）

---

## 📝 快速参考命令

### 基本测试

```bash
# 1. 查找端口
ls -l /dev/ttyACM*

# 2. 设置权限
sudo chmod 666 /dev/ttyACM0

# 3. 发送测试命令
python3 script/cmd_vel_forwarder.py --port /dev/ttyACM0 --vx 0.5 --rate 20

# 4. ROS2模式
python3 script/cmd_vel_forwarder.py --port /dev/ttyACM0 --ros2
```

### 查看日志

```bash
# 方法1：使用screen
screen /dev/ttyACM0 115200

# 方法2：使用项目脚本
python3 script/view_stm32_logs.py /dev/ttyACM0

# 方法3：使用cat（简单但可能乱码）
cat /dev/ttyACM0
```

---

## 💡 提示

1. **端口号可能变化**：每次重新插拔USB，端口号可能会变化（如从ttyACM0变成ttyACM1）
2. **使用符号链接**：可以创建符号链接固定端口名：
   ```bash
   sudo ln -s /dev/ttyACM0 /dev/stm32
   ```
3. **检查连接**：使用 `lsusb` 确认STM32已被识别
4. **测试顺序**：先测试基本通信，再集成ROS2

---

## 🎯 验证清单

部署完成后，确认：

- [ ] 可以找到USB端口（`/dev/ttyACM0`等）
- [ ] 端口权限已设置
- [ ] Python脚本可以运行
- [ ] 可以发送命令并看到STM32响应
- [ ] STM32日志显示数据接收正常
- [ ] 底盘可以响应命令（如果电机已连接）
- [ ] ROS2集成正常（如果使用ROS2）

---

## 📚 相关文档

- `docs/QUICK_TEST_GUIDE.md` - 快速测试指南
- `docs/RADAR_TEST_GUIDE.md` - Radar通信测试指南
- `docs/FIX_LOG_GARBLED.md` - 解决日志乱码问题
- `docs/DIAGNOSE_CHASSIS_NOT_MOVING.md` - 底盘不动问题诊断
