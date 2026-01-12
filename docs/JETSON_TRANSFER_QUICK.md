# Jetson文件传输 - 快速指南（针对你的配置）

## 🎯 你的SSH配置

```
Host: nuc-7913
HostName: 42.192.208.124
Port: 7913
User: nyu
```

---

## 📋 方法1：使用SCP传输（推荐）

### 在Windows PowerShell中

```powershell
# 1. 进入项目目录
cd C:\Users\zhuya\Desktop\robomaster\robo-electro-control\robomaster-control

# 2. 传输script目录到Jetson
# 注意：使用 -P 指定端口（大写P），不是小写p
scp -P 7913 -r script/ nyu@42.192.208.124:/home/nyu/robomaster-control/

# 系统会提示输入密码，输入Jetson的密码
```

**参数说明**：
- `-P 7913`：指定SSH端口（注意是大写P）
- `-r`：递归复制整个目录
- `script/`：要传输的本地目录
- `nyu@42.192.208.124`：用户名和IP地址
- `/home/nyu/robomaster-control/`：Jetson上的目标路径

### 如果使用SSH配置文件（更简单）

如果你在Windows上有SSH配置文件（`C:\Users\zhuya\.ssh\config`），可以添加：

```
Host nuc-7913
    HostName 42.192.208.124
    User nyu
    Port 7913
```

然后直接使用：

```powershell
scp -r script/ nuc-7913:/home/nyu/robomaster-control/
```

---

## 📋 方法2：测试SSH连接

### 先测试能否连接

```powershell
# 在Windows PowerShell中
ssh -p 7913 nyu@42.192.208.124

# 如果连接成功，会提示输入密码
# 输入密码后，应该能看到Jetson的命令行提示符
# 输入 exit 退出
```

**注意**：SSH端口参数是小写 `-p`，SCP端口参数是大写 `-P`

---

## 📋 方法3：使用WinSCP（图形界面）

### 设置

1. **打开WinSCP**
2. **新建会话**：
   - 文件协议：`SFTP`
   - 主机名：`42.192.208.124`
   - 端口号：`7913`
   - 用户名：`nyu`
   - 密码：输入你的密码
3. **保存会话**（可选，方便下次使用）
4. **登录**

### 传输文件

- 左侧：Windows文件系统
- 右侧：Jetson文件系统
- 拖拽 `script/` 目录从左侧到右侧的 `/home/nyu/robomaster-control/` 目录

---

## 📋 传输完成后的步骤

### 在Jetson上（通过SSH登录）

```bash
# 1. SSH登录到Jetson
ssh -p 7913 nyu@42.192.208.124

# 2. 进入项目目录
cd ~/robomaster-control

# 3. 确认文件已传输
ls -la script/

# 4. 运行设置脚本
chmod +x script/setup_jetson.sh
./script/setup_jetson.sh

# 5. 测试基本功能
python3 script/cmd_vel_forwarder.py --port /dev/ttyACM0 --vx 0.5 --rate 20
```

---

## 🔧 常见问题

### Q1: "Connection refused" 或 "Connection timed out"

**可能原因**：
- Jetson的SSH服务未启动
- 端口7913被防火墙阻止
- IP地址不正确

**解决方法**：
```bash
# 在Jetson上检查SSH服务
sudo systemctl status ssh

# 如果没有运行，启动它
sudo systemctl start ssh
sudo systemctl enable ssh
```

### Q2: "Permission denied"

**可能原因**：
- 密码错误
- 用户名错误

**解决方法**：
- 确认用户名是 `nyu`
- 确认密码正确
- 检查Jetson是否允许密码登录

### Q3: SCP命令找不到

**解决方法**：
```powershell
# Windows 10/11通常自带，检查
ssh -V

# 如果没有，安装OpenSSH客户端
# 在"设置" → "应用" → "可选功能" → 添加"OpenSSH客户端"
```

### Q4: 传输速度慢

**可能原因**：
- 网络连接问题
- 公网IP可能经过NAT/防火墙

**解决方法**：
- 如果可能，使用内网IP
- 或使用U盘直接传输

---

## 📝 快速命令参考

### 传输文件

```powershell
# 完整命令
scp -P 7913 -r script/ nyu@42.192.208.124:/home/nyu/robomaster-control/

# 如果配置了SSH config，可以使用别名
scp -r script/ nuc-7913:/home/nyu/robomaster-control/
```

### SSH登录

```powershell
# 完整命令
ssh -p 7913 nyu@42.192.208.124

# 如果配置了SSH config
ssh nuc-7913
```

### 传输单个文件

```powershell
# 传输单个文件（不需要 -r）
scp -P 7913 script/cmd_vel_forwarder.py nyu@42.192.208.124:/home/nyu/robomaster-control/script/
```

---

## 💡 提示

1. **记住端口号**：你的SSH端口是 `7913`，不是默认的 `22`
2. **SCP vs SSH端口参数**：
   - SCP：`-P 7913`（大写P）
   - SSH：`-p 7913`（小写p）
3. **首次连接**：可能会提示确认主机密钥，输入 `yes` 确认
4. **密码输入**：输入密码时不会显示字符，这是正常的

---

## 🎯 完整流程示例

```powershell
# ===== 在Windows PowerShell中 =====

# 1. 进入项目目录
cd C:\Users\zhuya\Desktop\robomaster\robo-electro-control\robomaster-control

# 2. 测试SSH连接
ssh -p 7913 nyu@42.192.208.124
# 输入密码，如果成功连接，输入 exit 退出

# 3. 传输文件
scp -P 7913 -r script/ nyu@42.192.208.124:/home/nyu/robomaster-control/
# 输入密码，等待传输完成

# ===== 在Jetson上（通过SSH） =====

# 4. SSH登录
ssh -p 7913 nyu@42.192.208.124

# 5. 进入项目目录
cd ~/robomaster-control

# 6. 运行设置脚本
chmod +x script/setup_jetson.sh
./script/setup_jetson.sh

# 7. 测试
python3 script/cmd_vel_forwarder.py --port /dev/ttyACM0 --vx 0.5 --rate 20
```
