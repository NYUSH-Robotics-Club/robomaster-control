# 系统性能调试指南

本文档总结了 RoboMaster 哨兵机器人导航系统的性能测量方法和基准数据。

---

## 📊 系统性能基准数据

### NUC 12 Pro 资源占用

| 指标 | 测量值 | 备注 |
|------|--------|------|
| CPU 型号 | i7-1260P (16核) | |
| CPU 峰值 | ~40% | Nav2 + SLAM 运行时 |
| CPU 平均 | ~36% | |
| 内存占用 | ~6.3 GB | |
| 网络带宽 | ~3.15 MB/s (RX) | 主要是点云数据 |

### 点云→LaserScan 处理耗时

| 指标 | 测量值 | 参考值 |
|------|--------|--------|
| 点云频率 | ~9-10 Hz | |
| LaserScan 频率 | ~7-9 Hz | |
| 丢帧率 | ~20-25% | < 10% 为佳 |
| 处理耗时 (mean) | ~15-18 ms/帧 | 2-5ms 为理想 |
| 处理耗时 (median) | ~14-15 ms/帧 | |
| 处理耗时 (p99) | ~150ms | 偶尔有尖峰 |

### Nav2 配置参数

| 参数 | 值 | 说明 |
|------|-----|------|
| max_vel_x | 0.26 m/s | 最大前进速度 |
| max_vel_y | 0.26 m/s | 最大横移速度 |
| max_speed_xy | 0.26 m/s | 最大合成速度 |
| min_speed_xy | 0.05 m/s | 最小速度（避免死区） |
| max_vel_theta | 0.0 rad/s | 旋转禁用 |
| acc_lim_x/y | 2.5 m/s² | 加速度限制 |

---

## 🔧 调试工具和方法

### 1. 端到端控制延迟测量 (`/cmd_vel` → 串口下发)

**脚本位置**: `script/cmd_vel_keyboard_fixed.py` 或 `script/cmd_vel_forwarder_updated.py`

```bash
# 基础延迟测量（每5秒输出统计）
python3 cmd_vel_keyboard_fixed.py --port /dev/ttyACM0 --ros2 --latency

# 详细模式（每条消息都打印延迟）
python3 cmd_vel_keyboard_fixed.py --port /dev/ttyACM0 --ros2 --latency --verbose
```

**需要另开终端发布 /cmd_vel**:
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" -r 20
```

**输出示例**:
```
[LATENCY] n=500 | min=0.12ms max=2.34ms | mean=0.45ms median=0.38ms | p95=1.02ms p99=1.85ms
```

**统计指标说明**:
- min/max: 最小/最大延迟
- mean: 平均延迟
- median: 中位数延迟
- p95/p99: 95/99 百分位延迟

---

### 2. 点云→LaserScan 节点耗时测量

**脚本位置**: `script/measure_pointcloud_latency.py`

```bash
# 基本用法
python3 measure_pointcloud_latency.py

# 指定话题
python3 measure_pointcloud_latency.py --cloud /cloud_registered --scan /scan

# 测量30秒后退出
python3 measure_pointcloud_latency.py --duration 30

# 每2秒报告一次
python3 measure_pointcloud_latency.py --interval 2
```

**快速检查频率**:
```bash
# 检查输入点云频率
ros2 topic hz /cloud_registered

# 检查输出 LaserScan 频率
ros2 topic hz /scan

# 检查延迟
ros2 topic delay /scan
```

**输出示例**:
```
----------------------------------------------------------------------
[14:32:15] Elapsed: 5.0s
  Cloud frequency:  10.2 Hz (51 msgs)
  Scan frequency:   10.0 Hz (50 msgs)
  Drop rate:        2.0%

  Cloud→Node latency: n=51 | min=1.23ms max=5.67ms | mean=2.45ms median=2.12ms
  Scan→Node latency:  n=50 | min=2.34ms max=8.90ms | mean=4.56ms median=4.12ms

  Estimated processing time: ~4.56ms (scan latency)
----------------------------------------------------------------------
```

---

### 3. 系统资源监控 (CPU / 内存 / 带宽)

**脚本位置**: `script/monitor_resources.sh`

```bash
# 持续监控
./monitor_resources.sh

# 监控 30 秒
./monitor_resources.sh --duration 30

# 单次快照
./monitor_resources.sh --once

# 每秒采样
./monitor_resources.sh --interval 1 --duration 60
```

**其他工具**:
```bash
# htop 交互式
htop

# 只看 ROS2 相关进程
htop -p $(pgrep -d, -f "ros2|python3|nav2")

# 网络带宽实时监控
sudo iftop -i enp114s0
nload enp114s0
```

**输出示例**:
```
========================================================================
 SUMMARY
========================================================================
 CPU Peak:      39.8%
 CPU Average:   36.6%
 Memory Peak:   6.3 GB
 Memory Avg:    6.29 GB
 RX Peak:       3.15 MB/s
 TX Peak:       0 MB/s
 Total BW Peak: 3.15 MB/s
========================================================================
```

---

### 4. ROS2 话题调试命令

```bash
# 列出所有话题
ros2 topic list

# 检查话题信息（发布者/订阅者数量）
ros2 topic info /scan

# 检查话题频率
ros2 topic hz /cmd_vel

# 查看话题内容
ros2 topic echo /cmd_vel

# 检查 TF 树
ros2 run tf2_tools view_frames

# 查看节点列表
ros2 node list

# 查看节点详情
ros2 node info /controller_server
```

---

### 5. STM32 串口日志过滤

脚本已优化，自动过滤二进制 radar 帧，只显示有效日志：

```
[STM32] GIM,568388,YAW_CSV,5736.00,5736.00,0,1209.28,0.8798,0.0000,0.0000,-0.00,0.0000,0.0000
[STM32] CMD,568802,RC,-0.000,-0.000,0.000
[STM32] [DEBUG][INFO] RADAR STATUS: buf_used=0/128 frames=16378 errors=0 valid=0
```

**有效日志前缀**:
- `GIM,` - 云台数据
- `CMD,` - 命令数据
- `CHA,` - 底盘数据
- `RAD,` - 雷达数据
- `[DEBUG]`, `[INFO]`, `[WARN]`, `[ERROR]` - 调试信息

---

## 📋 性能检查清单

### 启动前检查

- [ ] 确认所有话题已发布: `ros2 topic list | grep -E "scan|cloud|cmd_vel|odom"`
- [ ] 检查 TF 是否完整: `ros2 run tf2_tools view_frames`
- [ ] 确认串口连接: `ls /dev/ttyACM*`

### 运行时监控

```bash
# 在不同终端运行
# 终端1: 资源监控
./monitor_resources.sh --duration 60

# 终端2: 点云延迟
python3 measure_pointcloud_latency.py --duration 60

# 终端3: 控制延迟（如果在测试 Nav2）
python3 cmd_vel_keyboard_fixed.py --port /dev/ttyACM0 --ros2 --latency
```

### 性能指标参考

| 指标 | 良好 | 可接受 | 需优化 |
|------|------|--------|--------|
| 控制延迟 (mean) | < 1ms | 1-5ms | > 5ms |
| 点云处理耗时 | < 5ms | 5-20ms | > 20ms |
| 丢帧率 | < 5% | 5-20% | > 20% |
| CPU 峰值 | < 50% | 50-80% | > 80% |

---

## 🚀 优化建议

### 如果控制延迟过高
1. 检查串口波特率是否足够
2. 减少 STM32 日志输出频率
3. 优化 Python 脚本（使用 `perf_counter_ns`）

### 如果点云处理过慢
1. 降低点云分辨率
2. 调整 pointcloud_to_laserscan 参数
3. 检查 CPU 负载，关闭不必要的节点

### 如果丢帧率过高
1. 检查网络带宽
2. 调整 QoS 设置
3. 降低话题发布频率

---

## 📁 相关文件

```
script/
├── cmd_vel_keyboard_fixed.py      # 键盘控制 + 延迟测量
├── cmd_vel_forwarder_updated.py   # ROS2 转发 + 延迟测量
├── measure_pointcloud_latency.py  # 点云延迟测量
├── monitor_resources.sh           # 系统资源监控
└── view_stm32_logs.py             # STM32 日志查看

config/
└── nav2_params_optimized.yaml     # Nav2 参数配置
```

---

*文档生成时间: 2026-01-25*
