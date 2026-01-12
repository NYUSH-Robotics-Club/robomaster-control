# Jetson文件传输指南

## 🎯 目标
将Windows上的Python脚本传输到Jetson。

---

## 📋 方法1：使用SCP（需要SSH，推荐）

### 前提条件

1. **Jetson已开启SSH服务**
2. **知道Jetson的IP地址**
3. **Windows上可以使用SSH客户端**

### 步骤1：检查Jetson SSH服务

在Jetson上运行：

```bash
# 检查SSH服务是否运行
sudo systemctl status ssh

# 如果没有运行，启动SSH服务
sudo systemctl start ssh
sudo systemctl enable ssh  # 设置开机自启动
```

### 步骤2：找到Jetson的IP地址

在Jetson上运行：

```bash
# 方法1：使用ifconfig
ifconfig

# 方法2：使用ip命令
ip addr show

# 方法3：查看hostname -I
hostname -I

# 方法4：查看网络连接（如果有图形界面）
# 在设置 → 网络 中查看
```

**常见IP地址格式**：
- `192.168.1.100`
- `192.168.0.100`
- `10.0.0.100`

### 步骤3：在Windows上测试SSH连接

在Windows PowerShell中：

```powershell
# 测试SSH连接（替换IP和用户名）
ssh jetson@192.168.1.100

# 如果连接成功，会提示输入密码
# 输入Jetson的用户密码
# 如果看到命令行提示符，说明SSH正常
```

**如果Windows没有SSH客户端**：

Windows 10/11 通常自带SSH，如果没有：

```powershell
# 检查是否有SSH
ssh -V

# 如果没有，安装OpenSSH客户端
# 在"设置" → "应用" → "可选功能" → 添加"OpenSSH客户端"
```

### 步骤4：传输文件

在Windows PowerShell中：

```powershell
# 进入项目目录
cd C:\Users\zhuya\Desktop\robomaster\robo-electro-control\robomaster-control

# 传输script目录到Jetson
# 格式：scp -r <本地目录> <用户名>@<IP地址>:<远程路径>
scp -r script/ jetson@192.168.1.100:/home/jetson/robomaster-control/

# 系统会提示输入Jetson的密码
# 输入密码后开始传输
```

**参数说明**：
- `-r`：递归复制整个目录
- `script/`：要传输的本地目录
- `jetson@192.168.1.100`：Jetson的用户名和IP地址
- `/home/jetson/robomaster-control/`：Jetson上的目标路径

**替换为你的实际值**：
- `jetson` → 你的Jetson用户名（可能是 `nvidia`、`ubuntu` 等）
- `192.168.1.100` → 你的Jetson实际IP地址
- `/home/jetson/robomaster-control/` → 你想存放文件的路径

---

## 📋 方法2：使用U盘（最简单，不需要网络）

### 步骤

1. **在Windows上**：
   - 将 `script/` 目录复制到U盘

2. **在Jetson上**：
   - 插入U盘
   - 挂载U盘（通常自动挂载在 `/media/用户名/`）
   - 复制文件：
     ```bash
     # 创建目标目录
     mkdir -p ~/robomaster-control
     
     # 复制文件（替换U盘路径）
     cp -r /media/jetson/USB_NAME/script ~/robomaster-control/
     ```

---

## 📋 方法3：使用Git（如果Jetson可以访问Git仓库）

### 步骤

在Jetson上：

```bash
# 克隆仓库
cd ~
git clone <your_repo_url> robomaster-control
cd robomaster-control

# 或者如果已有仓库，拉取最新代码
cd ~/robomaster-control
git pull
```

---

## 📋 方法4：使用WinSCP（图形界面，Windows推荐）

### 步骤

1. **下载WinSCP**：
   - 访问：https://winscp.net/
   - 下载并安装

2. **连接Jetson**：
   - 打开WinSCP
   - 文件协议：选择 `SFTP`
   - 主机名：输入Jetson的IP地址（如 `192.168.1.100`）
   - 用户名：输入Jetson用户名（如 `jetson`）
   - 密码：输入Jetson密码
   - 点击"登录"

3. **传输文件**：
   - 左侧：Windows文件系统
   - 右侧：Jetson文件系统
   - 拖拽 `script/` 目录从左侧到右侧

---

## 📋 方法5：使用共享文件夹/Samba（适合频繁传输）

### 在Jetson上设置Samba服务器

```bash
# 安装Samba
sudo apt-get install samba

# 配置Samba（编辑配置文件）
sudo nano /etc/samba/smb.conf

# 在文件末尾添加：
[robomaster]
   path = /home/jetson/robomaster-control
   valid users = jetson
   read only = no

# 设置Samba用户密码
sudo smbpasswd -a jetson

# 重启Samba服务
sudo systemctl restart smbd
```

### 在Windows上访问

1. 打开"文件资源管理器"
2. 在地址栏输入：`\\192.168.1.100\robomaster`
3. 输入用户名和密码
4. 直接拖拽文件

---

## 🔧 常见问题

### Q1: "ssh: connect to host ... port 22: Connection refused"

**原因**：Jetson的SSH服务未启动

**解决方法**：
```bash
# 在Jetson上
sudo systemctl start ssh
sudo systemctl enable ssh
```

### Q2: "Permission denied (publickey,password)"

**原因**：密码错误或SSH配置问题

**解决方法**：
- 确认用户名和密码正确
- 检查Jetson是否允许密码登录（默认允许）

### Q3: 找不到Jetson的IP地址

**解决方法**：
```bash
# 在Jetson上
hostname -I
# 或
ip addr show | grep "inet "
```

### Q4: Windows没有SSH/SCP命令

**解决方法**：
- Windows 10/11：通常自带，检查 `ssh -V`
- 如果没有：安装OpenSSH客户端（在"可选功能"中）
- 或使用WinSCP（图形界面）

### Q5: 传输速度很慢

**可能原因**：
- 网络连接问题
- 使用WiFi而不是有线网络
- Jetson性能限制

**解决方法**：
- 使用有线网络连接
- 或使用U盘直接传输

---

## 📝 快速参考

### 最常用的方法（SCP）

```powershell
# Windows PowerShell
cd C:\Users\zhuya\Desktop\robomaster\robo-electro-control\robomaster-control
scp -r script/ jetson@192.168.1.100:/home/jetson/robomaster-control/
```

### 最简单的方法（U盘）

1. Windows：复制 `script/` 到U盘
2. Jetson：插入U盘，复制文件

### 最专业的方法（WinSCP）

1. 下载安装WinSCP
2. 连接Jetson（图形界面）
3. 拖拽文件传输

---

## 💡 推荐工作流程

1. **首次设置**：使用U盘或WinSCP（简单直接）
2. **日常开发**：使用SCP或Git（快速便捷）
3. **频繁传输**：设置Samba共享文件夹

---

## 🎯 下一步

文件传输完成后，在Jetson上：

```bash
# 1. 进入项目目录
cd ~/robomaster-control

# 2. 运行设置脚本
chmod +x script/setup_jetson.sh
./script/setup_jetson.sh

# 3. 测试
python3 script/cmd_vel_forwarder.py --port /dev/ttyACM0 --vx 0.5 --rate 20
```
