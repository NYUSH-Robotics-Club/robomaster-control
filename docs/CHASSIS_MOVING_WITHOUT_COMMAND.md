# 底盘在没有命令时也在动 - 问题排查

## 🎯 问题现象

即使没有运行Python脚本发送命令，底盘也在移动。

---

## 🔍 可能的原因

### 原因1：后台Python进程仍在运行（最常见）

**检查方法**：
```powershell
# 查看所有Python进程
tasklist | findstr python

# 或者更详细
wmic process where "name='python.exe'" get processid,commandline
```

**解决方法**：
```powershell
# 关闭所有Python进程
taskkill /F /IM python.exe

# 或者关闭特定进程（替换PID）
taskkill /F /PID <进程ID>
```

---

### 原因2：遥控器（RC）正在发送命令

**说明**：STM32的控制逻辑是：
1. **优先级1**：如果radar数据有效且新鲜（<500ms），使用radar模式
2. **优先级2**：否则，fallback到**遥控器（RC）模式**

**检查方法**：
- 检查遥控器是否已连接并上电
- 检查遥控器的摇杆是否在非零位置
- 检查遥控器的开关状态

**解决方法**：
- 将遥控器摇杆归中（所有通道归零）
- 或者断开遥控器连接

---

### 原因3：STM32还在使用最后接收到的有效命令

**说明**：代码中有500ms的超时保护：
- 如果radar数据在500ms内有效，STM32会继续使用该命令
- 如果超过500ms没有新数据，会fallback到RC模式

**检查方法**：
- 查看STM32日志，看是否显示 `RADAR TIMEOUT`
- 查看控制模式是否从 `RADAR` 切换到 `RC`

---

### 原因4：底盘控制器保持最后的速度命令

**说明**：如果底盘控制器没有收到停止命令，可能会保持最后的速度。

**检查方法**：
- 查看 `s_chassis_cmd.enabled` 状态
- 查看底盘控制器的 `running` 状态

---

## ✅ 解决步骤

### 步骤1：检查并关闭所有Python进程

```powershell
# 查看进程
tasklist | findstr python

# 如果有，关闭它们
taskkill /F /IM python.exe
```

### 步骤2：检查遥控器状态

- **断开遥控器连接**，或
- **将遥控器摇杆归中**，或
- **关闭遥控器电源**

### 步骤3：发送停止命令

如果Python脚本还在运行，发送零速度命令：

```powershell
python script/cmd_vel_forwarder.py --port COM9 --vx 0.0 --vy 0.0 --wz 0.0 --rate 20
```

然后按 `Ctrl+C` 停止。

### 步骤4：等待超时

如果之前发送了非零速度命令，等待500ms后，STM32应该会：
- 检测到radar数据超时
- 自动fallback到RC模式
- 如果RC也没有输入，底盘应该停止

---

## 🔧 代码逻辑说明

### 控制模式优先级

```c
// 1. 如果radar数据有效且新鲜（<500ms）
if (s_last_radar.valid && (now - s_last_radar.ts_ms <= 500u)) {
    // 使用radar模式
    radar_cmd_to_wheel_speeds(...);
} else {
    // 2. 否则，使用RC模式
    process_chassis_command(&s_last_rc, ...);
}
```

### 超时保护

```c
// 在 radar_cmd_to_wheel_speeds 中
if (!s_last_radar.valid || (now - s_last_radar.ts_ms > 500u)) {
    s_control_mode = CONTROL_MODE_RC;  // 切换到RC模式
    return;
}
```

---

## 📝 最佳实践

### 安全停止方法

1. **发送零速度命令**：
   ```powershell
   python script/cmd_vel_forwarder.py --port COM9 --vx 0.0 --vy 0.0 --wz 0.0 --rate 20
   ```

2. **等待500ms**（让STM32检测超时）

3. **停止Python脚本**（按 `Ctrl+C`）

4. **确认遥控器归中**

---

## 🐛 如果问题持续

### 检查清单

- [ ] 所有Python进程已关闭
- [ ] 遥控器已断开或归中
- [ ] 已发送零速度命令
- [ ] 已等待500ms超时
- [ ] STM32日志显示已切换到RC模式

### 如果还是不行

1. **重启STM32**（断电重上电）
2. **检查是否有其他程序在使用COM端口**
3. **查看STM32日志，确认当前控制模式**

---

## 💡 提示

- **安全第一**：测试时确保底盘架空，避免意外移动造成伤害
- **明确停止**：发送零速度命令后再停止Python脚本
- **检查RC**：确保遥控器不会意外发送命令
