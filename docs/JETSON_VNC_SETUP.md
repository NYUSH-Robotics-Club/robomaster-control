# Jetson VNC 远程桌面设置指南

## 🎯 目标
通过VNC在Windows上远程连接到Jetson的图形界面，运行RViz等GUI程序。

---

## 📋 步骤1：在Jetson上安装VNC服务器

### 方法1：使用TigerVNC（推荐）

```bash
# SSH登录到Jetson
ssh -p 7913 nyu@42.192.208.124

# 更新包列表
sudo apt-get update

# 安装TigerVNC服务器
sudo apt-get install -y tigervnc-standalone-server tigervnc-common

# 安装桌面环境（如果还没有）
sudo apt-get install -y ubuntu-desktop-minimal
# 或者轻量级桌面
sudo apt-get install -y xfce4 xfce4-goodies
```

### 方法2：使用TightVNC（备选）

```bash
sudo apt-get install -y tightvncserver
```

---

## 📋 步骤2：配置VNC服务器

### 首次设置VNC密码

```bash
# 设置VNC密码（用于连接认证）
vncserver

# 系统会提示：
# You will require a password to access your desktops.
# Password: (输入密码，至少6位)
# Verify: (再次输入密码)
# Would you like to enter a view-only password (y/n)? n (输入n)
```

**重要**：这个密码是VNC连接密码，不是系统登录密码。

### 停止默认启动的VNC服务器

```bash
# 停止刚才启动的服务器（我们稍后会配置自动启动）
vncserver -kill :1
```

---

## 📋 步骤3：配置VNC启动脚本

### 创建启动脚本

```bash
# 创建配置文件目录
mkdir -p ~/.vnc

# 创建启动脚本
nano ~/.vnc/xstartup
```

### 添加以下内容（根据你的桌面环境选择）

#### 选项A：使用XFCE（轻量级，推荐）

```bash
#!/bin/bash
unset SESSION_MANAGER
unset DBUS_SESSION_BUS_ADDRESS
[ -x /etc/vnc/xstartup ] && exec /etc/vnc/xstartup
[ -r $HOME/.Xresources ] && xrdb $HOME/.Xresources
x-window-manager &
startxfce4 &
```

#### 选项B：使用GNOME（完整桌面）

```bash
#!/bin/bash
unset SESSION_MANAGER
unset DBUS_SESSION_BUS_ADDRESS
[ -x /etc/vnc/xstartup ] && exec /etc/vnc/xstartup
[ -r $HOME/.Xresources ] && xrdb $HOME/.Xresources
gnome-session &
```

#### 选项C：使用Ubuntu默认桌面

```bash
#!/bin/bash
unset SESSION_MANAGER
unset DBUS_SESSION_BUS_ADDRESS
[ -x /etc/vnc/xstartup ] && exec /etc/vnc/xstartup
[ -r $HOME/.Xresources ] && xrdb $HOME/.Xresources
x-window-manager &
gnome-session --session=ubuntu &
```

### 设置执行权限

```bash
chmod +x ~/.vnc/xstartup
```

---

## 📋 步骤4：启动VNC服务器

### 启动VNC（显示编号:1，端口5901）

```bash
# 启动VNC服务器，分辨率1920x1080，24位色深
vncserver :1 -geometry 1920x1080 -depth 24

# 或者使用其他分辨率
vncserver :1 -geometry 1280x720 -depth 24
```

### 查看VNC状态

```bash
# 查看运行的VNC服务器
vncserver -list

# 应该看到类似：
# TigerVNC server sessions:
# X DISPLAY #     PROCESS ID
# :1              12345
```

---

## 📋 步骤5：在Windows上安装VNC客户端

### 方法1：使用TightVNC Viewer（推荐，免费）

1. **下载**：
   - 访问：https://www.tightvnc.com/download.php
   - 下载 "TightVNC Viewer for Windows"

2. **安装**：
   - 运行安装程序
   - 选择 "Viewer only"（只需要查看器）

3. **连接**：
   - 打开 TightVNC Viewer
   - 输入：`42.192.208.124:5901` 或 `42.192.208.124::7913`（如果VNC端口是7913）
   - 输入VNC密码
   - 点击连接

### 方法2：使用RealVNC Viewer（推荐，免费）

1. **下载**：
   - 访问：https://www.realvnc.com/download/viewer/
   - 下载 "VNC Viewer for Windows"

2. **连接**：
   - 打开 RealVNC Viewer
   - 输入：`42.192.208.124:5901`
   - 输入VNC密码

### 方法3：使用UltraVNC（备选）

1. **下载**：https://www.uvnc.com/downloads/ultravnc.html

---

## 📋 步骤6：配置防火墙（如果需要）

如果连接不上，可能需要开放VNC端口：

```bash
# 在Jetson上
# 检查防火墙状态
sudo ufw status

# 如果防火墙开启，开放VNC端口
sudo ufw allow 5901/tcp
# 或者如果使用其他端口
sudo ufw allow 5902/tcp
```

---

## 📋 步骤7：设置VNC开机自启动（可选）

### 创建systemd服务

```bash
sudo nano /etc/systemd/system/vncserver@.service
```

添加以下内容：

```ini
[Unit]
Description=Start TightVNC server at startup
After=syslog.target network.target

[Service]
Type=forking
User=nyu
PAMName=login
PIDFile=/home/nyu/.vnc/%H:%i.pid
ExecStartPre=-/usr/bin/vncserver -kill :%i > /dev/null 2>&1
ExecStart=/usr/bin/vncserver -depth 24 -geometry 1920x1080 :%i
ExecStop=/usr/bin/vncserver -kill :%i

[Install]
WantedBy=multi-user.target
```

启用服务：

```bash
# 启用服务（:1表示显示1）
sudo systemctl daemon-reload
sudo systemctl enable vncserver@1.service
sudo systemctl start vncserver@1.service

# 查看状态
sudo systemctl status vncserver@1.service
```

---

## 🔧 常见问题

### Q1: 连接失败 "Connection refused"

**解决方法**：
```bash
# 检查VNC是否运行
vncserver -list

# 检查端口是否监听
netstat -tlnp | grep 5901

# 重启VNC
vncserver -kill :1
vncserver :1 -geometry 1920x1080 -depth 24
```

### Q2: 连接后显示灰色屏幕或黑屏

**原因**：xstartup配置不正确

**解决方法**：
```bash
# 重新配置xstartup
nano ~/.vnc/xstartup

# 使用上面的XFCE配置（推荐）
# 然后重启VNC
vncserver -kill :1
vncserver :1 -geometry 1920x1080 -depth 24
```

### Q3: 分辨率太小

**解决方法**：
```bash
# 停止当前VNC
vncserver -kill :1

# 使用更高分辨率启动
vncserver :1 -geometry 2560x1440 -depth 24
# 或
vncserver :1 -geometry 1920x1080 -depth 24
```

### Q4: 性能慢或卡顿

**解决方法**：
1. 降低分辨率：`vncserver :1 -geometry 1280x720 -depth 24`
2. 降低色深：`vncserver :1 -geometry 1920x1080 -depth 16`
3. 使用有线网络而不是WiFi
4. 关闭不必要的视觉效果

### Q5: 忘记VNC密码

**解决方法**：
```bash
# 删除密码文件，重新设置
rm ~/.vnc/passwd
vncserver
```

---

## 📝 快速参考命令

### 启动VNC

```bash
# 启动（显示1，分辨率1920x1080）
vncserver :1 -geometry 1920x1080 -depth 24

# 停止
vncserver -kill :1

# 查看状态
vncserver -list

# 重启
vncserver -kill :1
vncserver :1 -geometry 1920x1080 -depth 24
```

### 连接信息

- **地址**：`42.192.208.124:5901`
- **端口说明**：
  - 显示:1 → 端口5901
  - 显示:2 → 端口5902
  - 显示:N → 端口590N

---

## 🎯 完整设置流程（复制粘贴）

```bash
# ===== 在Jetson上（SSH登录后） =====

# 1. 安装VNC服务器
sudo apt-get update
sudo apt-get install -y tigervnc-standalone-server tigervnc-common xfce4 xfce4-goodies

# 2. 设置VNC密码
vncserver
# 输入密码（至少6位）

# 3. 停止默认启动
vncserver -kill :1

# 4. 配置启动脚本
mkdir -p ~/.vnc
cat > ~/.vnc/xstartup << 'EOF'
#!/bin/bash
unset SESSION_MANAGER
unset DBUS_SESSION_BUS_ADDRESS
[ -x /etc/vnc/xstartup ] && exec /etc/vnc/xstartup
[ -r $HOME/.Xresources ] && xrdb $HOME/.Xresources
x-window-manager &
startxfce4 &
EOF

chmod +x ~/.vnc/xstartup

# 5. 启动VNC
vncserver :1 -geometry 1920x1080 -depth 24

# 6. 查看状态
vncserver -list
```

### 在Windows上

1. **下载并安装** TightVNC Viewer 或 RealVNC Viewer
2. **打开VNC Viewer**
3. **输入地址**：`42.192.208.124:5901`
4. **输入VNC密码**
5. **连接**

---

## 💡 提示

1. **多个VNC会话**：可以启动多个显示（:1, :2, :3），每个对应不同端口
2. **安全建议**：VNC密码不要和系统密码相同
3. **性能优化**：如果网络慢，降低分辨率和色深
4. **自动启动**：设置systemd服务后，重启Jetson会自动启动VNC

---

## 🎯 连接后运行RViz

连接成功后，在VNC窗口中：

```bash
# 打开终端
# 初始化ROS环境
source /opt/ros/humble/setup.bash
cd ~/nav_ws
source install/setup.bash

# 运行RViz（使用Nav2默认配置）
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz

# 或者运行不带配置的RViz（需要手动加载配置）
ros2 run rviz2 rviz2

# 或者运行你的Nav2脚本（如果脚本中包含RViz启动）
./your_nav2_script.sh
```

**说明**：
- `-d` 参数指定RViz配置文件
- `$(ros2 pkg prefix nav2_bringup)` 会自动解析Nav2包的安装路径
- `nav2_default_view.rviz` 是Nav2提供的默认可视化配置

现在你可以在VNC窗口中看到RViz的图形界面了！
