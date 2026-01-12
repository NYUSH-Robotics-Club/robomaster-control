# 解决SCP传输错误

## 🎯 问题

```
No such file or directory: /home/nyu/robomaster-control/
```

**原因**：目标目录不存在，需要先创建。

---

## ✅ 解决方法

### 方法1：先创建目录，再传输（推荐）

#### 步骤1：SSH登录到Jetson

```powershell
# 在Windows PowerShell中
ssh -p 7913 nyu@42.192.208.124
```

输入密码登录。

#### 步骤2：在Jetson上创建目录

```bash
# 创建项目目录
mkdir -p ~/robomaster-control

# 确认创建成功
ls -la ~/robomaster-control

# 退出SSH
exit
```

#### 步骤3：传输文件

```powershell
# 回到Windows PowerShell
cd C:\Users\zhuya\Desktop\robomaster\robo-electro-control\robomaster-control

# 传输文件
scp -P 7913 -r script/ nyu@42.192.208.124:/home/nyu/robomaster-control/
```

---

### 方法2：直接传输到home目录，然后移动

#### 步骤1：传输到home目录

```powershell
# 直接传输到用户主目录
scp -P 7913 -r script/ nyu@42.192.208.124:~/
```

#### 步骤2：SSH登录整理文件

```powershell
ssh -p 7913 nyu@42.192.208.124
```

```bash
# 创建项目目录
mkdir -p ~/robomaster-control

# 移动script目录
mv ~/script ~/robomaster-control/

# 确认
ls -la ~/robomaster-control/
```

---

### 方法3：使用一条命令（最方便）

在Windows PowerShell中，使用SSH执行远程命令创建目录：

```powershell
# 先创建目录（通过SSH执行命令）
ssh -p 7913 nyu@42.192.208.124 "mkdir -p ~/robomaster-control"

# 然后传输文件
scp -P 7913 -r script/ nyu@42.192.208.124:/home/nyu/robomaster-control/
```

---

## 🔧 关于密码错误

如果遇到 "Permission denied"：

1. **确认密码正确**：注意大小写
2. **确认用户名正确**：应该是 `nyu`
3. **如果多次失败**：可能需要检查Jetson的SSH配置

---

## 📝 完整流程（推荐）

```powershell
# ===== 在Windows PowerShell中 =====

# 1. 先创建目录（一条命令）
ssh -p 7913 nyu@42.192.208.124 "mkdir -p ~/robomaster-control"

# 2. 传输文件
cd C:\Users\zhuya\Desktop\robomaster\robo-electro-control\robomaster-control
scp -P 7913 -r script/ nyu@42.192.208.124:/home/nyu/robomaster-control/

# 3. 验证传输（可选）
ssh -p 7913 nyu@42.192.208.124 "ls -la ~/robomaster-control/script/"
```

---

## 💡 提示

- **`mkdir -p`**：如果目录已存在不会报错，如果不存在会创建
- **`~`**：代表用户主目录（`/home/nyu/`）
- **先创建目录再传输**：避免路径错误

---

## 🎯 快速命令（复制粘贴）

```powershell
# 创建目录
ssh -p 7913 nyu@42.192.208.124 "mkdir -p ~/robomaster-control"

# 传输文件（在项目目录中运行）
scp -P 7913 -r script/ nyu@42.192.208.124:/home/nyu/robomaster-control/
```
