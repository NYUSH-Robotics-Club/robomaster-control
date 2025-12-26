# 编码器值读取工具使用说明

## 功能说明

这个Python脚本用于从USB CDC串口读取并过滤云台编码器数据，只显示包含`ENCODER`标签的行，过滤掉其他调试信息。

## 数据格式

串口输出格式：
```
ENCODER,timestamp_ms,yaw_raw,pitch_raw,yaw_target,pitch_target
```

示例：
```
ENCODER,12345,2711,2500,2800.0,2450.0
```

字段说明：
- `timestamp_ms`: 系统时间戳（毫秒）
- `yaw_raw`: Yaw轴当前编码器值（0-8191）
- `pitch_raw`: Pitch轴当前编码器值（0-8191）
- `yaw_target`: Yaw轴目标角度（刻度）
- `pitch_target`: Pitch轴目标角度（刻度）

## 安装依赖

```bash
# macOS/Linux
pip3 install pyserial

# Windows
pip install pyserial
```

## 使用方法

### 基本使用（自动查找串口）

```bash
cd /Users/pengyue/Codespace/robomaster-control/scripts
python3 read_encoder.py
```

### 指定串口设备

```bash
# macOS
python3 read_encoder.py /dev/cu.usbmodem14201

# Linux
python3 read_encoder.py /dev/ttyACM0

# Windows
python3 read_encoder.py COM3
```

### 保存数据到CSV

```bash
python3 read_encoder.py --save encoder_data.csv
```

### 自定义波特率

```bash
python3 read_encoder.py --baud 921600
```

### 完整示例

```bash
python3 read_encoder.py /dev/cu.usbmodem14201 --save data.csv --baud 115200
```

## 输出示例

```
找到USB CDC设备: /dev/cu.usbmodem14201 (STM32 Virtual ComPort)
正在连接到 /dev/cu.usbmodem14201 (波特率: 115200)...
已连接！正在读取ENCODER数据... (按Ctrl+C退出)

时间          Yaw原始  Yaw目标  Yaw误差 | Pitch原始 Pitch目标 Pitch误差
--------------------------------------------------------------------------------
     12.34s      2711   2800.0     89.0 |      2500   2450.0    -50.0
     12.39s      2720   2800.0     80.0 |      2495   2450.0    -45.0
     12.44s      2730   2800.0     70.0 |      2490   2450.0    -40.0
     12.49s      2740   2800.0     60.0 |      2485   2450.0    -35.0
```

## 故障排查

### 找不到串口设备

1. 确认STM32已通过USB连接到电脑
2. 检查USB CDC是否已初始化（代码中的`MX_USB_DEVICE_Init()`）
3. macOS: 查看 `/dev/cu.*` 设备列表
   ```bash
   ls -l /dev/cu.*
   ```
4. Linux: 查看 `/dev/ttyACM*` 设备列表
   ```bash
   ls -l /dev/ttyACM*
   ```

### 权限错误（Linux）

```bash
# 添加用户到dialout组
sudo usermod -a -G dialout $USER
# 重新登录后生效
```

### 数据乱码

1. 检查波特率是否正确（默认115200）
2. 确认STM32端的USB CDC配置
3. 尝试重新插拔USB线

### 没有数据输出

1. 确认云台已启用（`s_last_cmd.enabled == true`）
2. 检查代码中的CDC输出是否正常（gimbal_controller.c 第278-293行）
3. 使用其他串口工具（如screen、minicom）验证是否有任何输出

## 代码修改位置

编码器输出代码位于：
- **文件**: `application/gimbal/gimbal_controller.c`
- **行数**: 278-293
- **更新频率**: 20Hz（每50ms）

如需修改输出频率：
```c
if (now - last_encoder_print >= 50) {  // 改为100则为10Hz
```

如需添加更多字段：
```c
USB_CDC_Printf("ENCODER,%lu,%d,%d,%.2f,%.2f,%d,%d\r\n",
               now,
               yaw->angle_raw,
               pitch->angle_raw,
               yaw->angle_target,
               pitch->angle_target,
               yaw->speed_rpm,      // 新增：速度反馈
               pitch->speed_rpm);   // 新增：速度反馈
```

## 注意事项

1. **数据速率**: 当前配置为20Hz，不会影响控制性能
2. **CSV文件**: 长时间运行会生成较大的CSV文件，建议定期清理
3. **环绕处理**: 脚本自动处理GM6020的0-8191环绕边界
4. **实时性**: 显示有约50ms的延迟（取决于USB CDC缓冲和Python处理）

## 高级用法

### 实时绘图（需要matplotlib）

```bash
pip3 install matplotlib

# 创建绘图脚本（TODO）
python3 read_encoder.py --save data.csv &
python3 plot_encoder.py data.csv
```

### 过滤其他数据

如果需要同时查看YAW_CSV数据，修改脚本第107行：
```python
if line.startswith('ENCODER') or line.startswith('YAW_CSV'):
```

## 相关文件

- **编码器输出代码**: `application/gimbal/gimbal_controller.c`
- **读取脚本**: `scripts/read_encoder.py`
- **数据结构**: `modules/motor/gm6020_motor.h` (GM6020_MotorContext)

---

**创建日期**: 2025-12-25
**作者**: Claude & PengYue
