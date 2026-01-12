# COM端口单程序使用说明

## 🎯 问题

在Windows上，**一个COM端口只能被一个程序独占打开**。如果同时运行两个程序尝试打开同一个COM端口，第二个程序会报错：

```
Failed to open serial port: could not open port 'COM9': OSError(22, '信号灯超时时间已到', None, 121)
```

---

## ✅ 解决方案

### 方案1：只运行 `cmd_vel_forwarder.py`（推荐）

**`cmd_vel_forwarder.py` 已经内置了日志查看功能**，你不需要单独运行日志查看脚本！

```powershell
# 只需要运行这一个程序
python script/cmd_vel_forwarder.py --port COM9 --vx 0.5 --vy 0.0 --wz 0.0 --rate 20
```

**这个程序会同时：**
- ✅ 发送速度命令到STM32
- ✅ 自动接收并显示STM32的日志
- ✅ 过滤掉二进制数据，只显示可读的文本日志

**输出示例：**
```
Sending frames at 20.0 Hz to COM9
Sent vx=0.500 vy=0.000 wz=0.000
STM32> [DEBUG][INFO] RADAR OK: vx=0.500 vy=0.000 wz=0.000 frames=123 errors=0
STM32> CMD,12345,RADAR,IN:0.500,0.000,0.000,OUT:0.500,0.000,0.000,sp0:10.00,sp1:10.00
Sent vx=0.500 vy=0.000 wz=0.000
...
```

---

### 方案2：使用两个不同的COM端口（不推荐）

如果你的STM32支持多个虚拟COM端口（很少见），可以：
- COM端口1：用于发送命令
- COM端口2：用于查看日志

但通常STM32只有一个USB CDC虚拟COM端口，所以这个方案不适用。

---

### 方案3：使用串口转发工具（高级）

可以使用虚拟串口工具（如com0com）创建虚拟串口对，但这通常不必要，因为方案1已经足够。

---

## 📝 最佳实践

### 推荐工作流程

1. **只运行一个程序**：
   ```powershell
   python script/cmd_vel_forwarder.py --port COM9 --vx 0.5 --rate 20
   ```

2. **如果需要查看更详细的日志**：
   - 使用 `--rate` 参数调整发送频率
   - 日志会自动显示在同一个终端窗口

3. **如果需要停止**：
   - 按 `Ctrl+C` 停止程序
   - 程序会自动关闭串口连接

---

## 🔧 常见问题

### Q1: 为什么不能同时运行两个程序？

**原因**：Windows串口驱动设计为独占访问模式，一个COM端口在同一时间只能被一个程序打开。这是操作系统的限制，不是程序的问题。

### Q2: 如果我想单独查看日志怎么办？

**回答**：使用 `view_stm32_logs_windows.py`，但**必须先关闭** `cmd_vel_forwarder.py`。

**步骤**：
1. 停止 `cmd_vel_forwarder.py`（按 `Ctrl+C`）
2. 等待几秒确保端口释放
3. 运行 `python script/view_stm32_logs_windows.py COM9`

### Q3: 如何确认端口是否被占用？

**检查方法**：
```powershell
# 查看所有Python进程
tasklist | findstr python

# 如果看到多个python.exe进程，可能需要关闭它们
taskkill /F /IM python.exe
```

### Q4: 程序显示"端口被占用"怎么办？

**解决方法**：
1. 关闭所有可能使用该端口的程序
2. 重新插拔USB线
3. 等待几秒后重试

---

## 💡 提示

- **一个程序就够了**：`cmd_vel_forwarder.py` 已经包含了日志查看功能
- **如果输出太多**：脚本已经自动过滤了二进制数据，只显示可读日志
- **如果需要更清晰的输出**：可以修改脚本的日志过滤逻辑

---

## 🎯 总结

**记住**：在Windows上，一个COM端口只能被一个程序使用。

**推荐做法**：只运行 `cmd_vel_forwarder.py`，它会同时处理发送命令和接收日志。
