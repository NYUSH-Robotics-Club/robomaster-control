# 雷达 (NUC) 外部控制集成方案

## 概述

已为 STM32 固件集成了完整的 **USB CDC 雷达控制模块**，支持 NUC/外部计算机通过 USB 发送自主运动指令 (`vx, vy, wz`)，实现底盘自动运动。

---

## 核心特性

| 特性 | 说明 |
|------|------|
| **协议帧** | `[0xA5][0x5A][vx:4B][vy:4B][wz:4B][CRC8:1B]` (14 字节) |
| **安全机制** | 环形缓冲 + 帧同步 + CRC8 校验 + 200ms 超时 |
| **优先级** | 雷达 > RC 遥控（自动切换） |
| **通信** | USB CDC (Linux: `/dev/ttyACM0`, Windows: `COM*`) |
| **主任务集成** | 已在 `main.c` 调用 `RadarComm_Task()` |

---

## 文件清单

### 新增文件
- `modules/radar_comm/radar_comm.h` - 协议定义与 API
- `modules/radar_comm/radar_comm.c` - 实现（环形缓冲 + CRC + 帧解析）
- `script/test_radar_comm.py` - NUC 端测试脚本

### 修改文件
- `message_center/message_center.h` - 新增 `TOPIC_RADAR_CMD`
- `application/cmd/cmd_controller.c` - 订阅 + 优先级控制 + swerve 映射
- `CMakeLists.txt` - 编译配置
- `Src/main.c` - 初始化 + 任务集成

---

## 协议规范

### 帧格式（14 字节，小端）

```
Byte  0      1      2-5      6-9      10-13    13
     [0xA5][0x5A][  vx   ][  vy   ][  wz   ][CRC8]
     同步1  同步2  float   float   float   校验
```

### 范围
- `vx, vy`: -10.0 ~ +10.0 m/s
- `wz`: -π ~ +π rad/s

### CRC8
多项式：`0x07`，计算前 13 字节，结果追加在第 14 字节

---

## 固件端集成

### 初始化（已在 `main.c` 中）
```c
RadarComm_Init();  // 初始化环形缓冲
```

### 主循环（已在 `main.c` 中）
```c
RadarComm_Task();  // 处理 USB 数据帧（200Hz）
```

### 命令处理（已在 `cmd_controller.c` 中）
```c
// 自动订阅 TOPIC_RADAR_CMD
MsgCenter_Subscribe(TOPIC_RADAR_CMD, on_radar_update, NULL);

// 自动优先级选择：
// - 若雷达数据有效且 <500ms，使用雷达指令
// - 否则回退到 RC 遥控
```

### Swerve 映射（已在 `cmd_controller.c` 中）
```c
// 将 (vx, vy, wz) 映射到转向模块角度与驱动速度
// 当前为简化版本（需按实际轮子位置调整）
radar_cmd_to_wheel_speeds(vx, vy, wz, timestamp);
```

---

## 安全机制详解

### 1. 环形缓冲（Ring Buffer）
- **问题**：USB CDC 可能分包（4B + 8B 而非 12B）
- **解决**：128 字节环形缓冲区，字节级别接收
- **ISR**：只写数据，**不做解析**
- **主任务**：做帧同步 + CRC 验证

### 2. 帧同步（Frame Sync）
- 搜索魔数对 `0xA5 0x5A`
- 若丢包导致字节错位，自动恢复（继续搜索下一个同步对）

### 3. CRC8 校验
- 检测 EMI（电磁干扰）导致的比特翻转
- 校验失败则丢弃该帧，继续搜索下一个

### 4. 超时保护
- 若 200ms 未收到有效帧，标记 `valid = 0`
- 命令处理层再次检查（500ms 超时），触发 fallback 到 RC

### 5. 优先级控制
```
if (radar_valid && timestamp_fresh) {
    使用雷达速度指令
    gimbal/shooter 仍受 RC 控制
} else {
    回退到 RC 遥控（原始行为）
}
```

---

## NUC 端测试

### 前提
1. STM32 已编译刷入（支持 USB CDC）
2. NUC 通过 USB 连接 STM32
3. 安装 `pyserial`：
   ```bash
   pip install pyserial
   ```

### 快速测试

#### 方式 1：运行自动测试序列
```bash
python3 script/test_radar_comm.py /dev/ttyACM0 --test-seq --duration 10
```

这会依次执行：
1. 前进 2s（vx=0.5 m/s）
2. 左平移 2s（vy=0.5 m/s）
3. 原地转 2s（wz=0.5 rad/s）
4. 混合运动 2s（vx=0.3, vy=0.2, wz=0.1）
5. 停止 0.5s

#### 方式 2：单次发送
```bash
# 前进 0.5 m/s，发送 1 次
python3 script/test_radar_comm.py /dev/ttyACM0 --vx 0.5 --count 1

# 左转 0.5 rad/s，发送 10 次（0.1s 间隔）
python3 script/test_radar_comm.py /dev/ttyACM0 --wz 0.5 --count 10 --interval 0.1
```

#### 方式 3：手工发送（在 Python 交互式终端）
```python
from script.test_radar_comm import encode_radar_cmd
import serial

port = serial.Serial('/dev/ttyACM0', 115200)
frame = encode_radar_cmd(0.5, 0.0, 0.0)  # 前进
port.write(frame)
port.close()
```

### 预期行为
- 固件收到有效帧后，底盘执行相应运动
- 如果运动异常，检查 **坐标系** 和 **轮子位置参数**（见下节）

---

## 自定义参数

### Swerve 轮子配置（`cmd_controller.c` 中 `radar_cmd_to_wheel_speeds()`）

当前假设（需按实际硬件调整）：
```c
const float L = 0.15f;  // 前后轮间距 (m)
const float W = 0.15f;  // 左右轮间距 (m)
const float r = 0.05f;  // 轮子半径 (m)
```

**调整步骤**：
1. 测量实际机器人尺寸
2. 修改上述三个常数
3. 重新编译刷入

### 超时阈值（`radar_comm.c` 中）

默认：
```c
#define RADAR_DATA_TIMEOUT_MS 200u  // 模块内超时
```

命令处理层（`cmd_controller.c`）：
```c
if (now - s_last_radar.ts_ms > 500u) {
    // 回退到 RC
}
```

---

## 故障排查

| 问题 | 原因 | 解决方案 |
|------|------|---------|
| USB 端口未找到 | 驱动/连接问题 | 检查 `dmesg`，确保 STM32 枚举为 CDC ACM 设备 |
| 帧接收但底盘不动 | 坐标系错误或轮子参数 | 修改 `L, W, r` 或检查坐标变换 |
| 运动方向反 | 电机方向或坐标系约定反 | 调整 swerve 映射中的 +/- 符号 |
| 频繁超时丢包 | USB 线质量差或干扰 | 更换 USB 线，检查 EMI |
| CRC 校验失败 | EMI 导致字节损坏 | 隔离 USB 线，远离电机 |

---

## 构建 & 编译

```bash
cd robomaster-control
cmake -S . -B build -DROBOT_TYPE=sentry_swerve
cmake --build build
```

如编译失败，检查：
- `radar_comm.h` 包含正确
- `CMakeLists.txt` 已添加 `.c` 文件
- 无重复定义或缺失头文件

---

## 架构图

```
NUC/PC (pyserial)
    |
    v
  USB CDC
    |
    v
RadarComm_RxCallback() --> Ring Buffer (128B)
                              |
                              v
                        RadarComm_Task() [main loop]
                          (帧同步 + CRC 校验)
                              |
                              v
                         TOPIC_RADAR_CMD
                              |
                              v
                   MsgCenter_Publish()
                              |
                              v
                  on_radar_update() [cmd_controller]
                              |
                              +---> RC 优先级检查
                              |
                              v (radar_valid && fresh)
                    radar_cmd_to_wheel_speeds()
                              |
                              v
                        s_chassis_cmd
                              |
                              v
                   MsgCenter_Publish(TOPIC_CHASSIS_CMD)
                              |
                              v
                    Chassis_Controller
                              |
                              v
                    Motor drivers (CAN)
                              |
                              v
                         底盘运动
```

---

## 下一步

1. **物理测试**：连接 NUC，运行 `test_radar_comm.py`，观察底盘运动
2. **参数微调**：根据实际运动调整 `L, W, r` 及 swerve 映射系数
3. **高级功能**（可选）：
   - 速度限幅（防止电机过载）
   - 加速度平滑（避免突变）
   - 融合 IMU 反馈（闭环控制）
   - 多雷达冗余

---

## 技术支持

如有问题，检查：
- 日志输出（`LOG_TAG_CMD`）确认雷达数据接收状态
- USB 抓包或串口监听工具确认帧完整性
- 电机反馈（CAN 报文）确认指令下发

