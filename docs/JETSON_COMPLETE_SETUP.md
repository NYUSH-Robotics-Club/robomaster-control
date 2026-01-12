# Jetson完整部署流程 - 复制粘贴版

## 🎯 完整流程（按顺序执行）

---

## 步骤1：在Windows PowerShell中创建目录

```powershell
ssh -p 7913 nyu@42.192.208.124 "mkdir -p ~/robomaster-control"
```

**说明**：输入密码，创建目标目录。

---

## 步骤2：传输文件

```powershell
cd C:\Users\zhuya\Desktop\robomaster\robo-electro-control\robomaster-control
scp -P 7913 -r script/ nyu@42.192.208.124:/home/nyu/robomaster-control/
```

**说明**：输入密码，等待传输完成。

---

## 步骤3：SSH登录到Jetson

```powershell
ssh -p 7913 nyu@42.192.208.124
```

**说明**：输入密码，登录到Jetson。

---

## 步骤4：在Jetson上验证文件

```bash
cd ~/robomaster-control
ls -la script/
```

**预期输出**：应该看到 `cmd_vel_forwarder.py` 等文件。

---

## 步骤5：安装依赖

```bash
sudo apt-get update
sudo apt-get install -y python3 python3-pip python3-serial screen
pip3 install pyserial
```

---

## 步骤6：设置串口权限

```bash
sudo usermod -a -G dialout $USER
newgrp dialout
```

---

## 步骤7：查找STM32端口

```bash
ls -l /dev/ttyACM*
```

**说明**：记下端口号，通常是 `/dev/ttyACM0`。

---

## 步骤8：设置端口权限（临时）

```bash
sudo chmod 666 /dev/ttyACM0
```

**说明**：替换 `ttyACM0` 为你的实际端口号。

---

## 步骤9：运行设置脚本

```bash
cd ~/robomaster-control
chmod +x script/setup_jetson.sh
./script/setup_jetson.sh
```

---

## 步骤10：测试基本通信

```bash
python3 script/cmd_vel_forwarder.py --port /dev/ttyACM0 --vx 0.5 --vy 0.0 --wz 0.0 --rate 20
```

**说明**：
- 按 `Ctrl+C` 停止
- 应该看到 `Sent vx=0.500...` 和 `STM32> [DEBUG][INFO] RADAR OK...` 输出

---

## 步骤11：ROS2集成（如果需要）

### 11.1 启动ROS2节点

```bash
python3 script/cmd_vel_forwarder.py --port /dev/ttyACM0 --ros2
```

### 11.2 在另一个终端发送ROS2命令

```bash
# 打开新的SSH会话或新终端
ssh -p 7913 nyu@42.192.208.124

# 发送测试命令
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# 或持续发送（20Hz）
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

---

## 📋 完整命令清单（一次性复制）

### Windows PowerShell部分

```powershell
# 1. 创建目录
ssh -p 7913 nyu@42.192.208.124 "mkdir -p ~/robomaster-control"

# 2. 传输文件
cd C:\Users\zhuya\Desktop\robomaster\robo-electro-control\robomaster-control
scp -P 7913 -r script/ nyu@42.192.208.124:/home/nyu/robomaster-control/

# 3. SSH登录
ssh -p 7913 nyu@42.192.208.124
```

### Jetson上执行（登录后）

```bash
# 4. 验证文件
cd ~/robomaster-control
ls -la script/

# 5. 安装依赖
sudo apt-get update
sudo apt-get install -y python3 python3-pip python3-serial screen
pip3 install pyserial

# 6. 设置权限
sudo usermod -a -G dialout $USER
newgrp dialout

# 7. 查找端口
ls -l /dev/ttyACM*

# 8. 设置端口权限（替换ttyACM0为实际端口）
sudo chmod 666 /dev/ttyACM0

# 9. 运行设置脚本
cd ~/robomaster-control
chmod +x script/setup_jetson.sh
./script/setup_jetson.sh

# 10. 测试
python3 script/cmd_vel_forwarder.py --port /dev/ttyACM0 --vx 0.5 --vy 0.0 --wz 0.0 --rate 20
```

---

## 🔧 常见问题快速解决

### 如果端口不是ttyACM0

```bash
# 查找所有串口
ls -l /dev/ttyACM* /dev/ttyUSB*

# 使用找到的端口，例如ttyACM1
python3 script/cmd_vel_forwarder.py --port /dev/ttyACM1 --vx 0.5 --rate 20
```

### 如果权限错误

```bash
# 临时解决
sudo chmod 666 /dev/ttyACM0

# 永久解决（需要重新登录）
sudo usermod -a -G dialout $USER
# 然后重新SSH登录
```

### 如果找不到Python模块

```bash
pip3 install pyserial
# 或
sudo apt-get install python3-serial
```

---

## ✅ 验证清单

完成后，确认：

- [ ] 文件已传输到Jetson
- [ ] 依赖已安装
- [ ] 端口权限已设置
- [ ] 可以运行Python脚本
- [ ] 可以看到STM32日志输出
- [ ] 底盘可以响应命令（如果电机已连接）

---

## 🎯 下一步

测试成功后，可以：

1. **集成到ROS2工作空间**
2. **创建启动文件**
3. **设置开机自启动**（参考 `docs/JETSON_DEPLOYMENT_GUIDE.md`）
