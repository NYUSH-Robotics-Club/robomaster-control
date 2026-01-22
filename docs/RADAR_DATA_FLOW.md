# Radar数据流完整解析

## 数据流图

```
Python脚本 (keyboard/ROS2)
    ↓
发送: [0xA5][0x5A][vx:4B][vy:4B][wz:4B][CRC8:1B] (15字节)
    ↓
STM32 USB CDC接收
    ↓
[RadarComm] ─────────────────────────────────────
    │ RadarComm_RxCallback (ISR)
    │ └─> 写入Ring Buffer (原始字节数据)
    │
    └─> RadarComm_Task (主task)
        ├─ 解析Sync: 0xA5 0x5A
        ├─ 提取vx/vy/wz (3个float)
        ├─ 校验CRC8
        └─> 发布 TOPIC_RADAR_CMD 消息
    ↓
[MessageCenter] ───────────────────────────────────
    └─> 触发订阅者回调: on_radar_update()
    ↓
[CmdController] ────────────────────────────────────
    │ on_radar_update() [消息回调]
    │ └─> 保存: s_last_radar (vx, vy, wz, ts_ms, valid)
    │
    └─> CmdController_Task() [主循环]
        ├─ 检查: radar.valid && ts_ms < 500ms
        ├─ YES: 调用 radar_cmd_to_wheel_speeds()
        │       ├─ 平滑过滤 (RADAR_SMOOTH_ALPHA=0.2)
        │       ├─ 加速度限制 (RADAR_MAX_DELTA_V=0.05)
        │       └─> 转换为舵轮角度 + 驱动转速
        │       └─> 发布 TOPIC_CHASSIS_CMD
        └─ NO: 回到RC模式
    ↓
[ChassisController] ──────────────────────────────
    └─> 接收 TOPIC_CHASSIS_CMD
        └─> 计算PID输出
        └─> 发送电机控制电流
    ↓
电机执行
```

## 关键数据结构

### 1. Radar接收数据 (radar_comm.h)
```c
typedef struct {
    float vx;        // m/s - 前进速度
    float vy;        // m/s - 侧向速度
    float wz;        // rad/s - 旋转速度
    uint32_t ts_ms;  // HAL_GetTick() 时间戳
    uint8_t valid;   // 1=有效, 0=超时/错误
} Radar_Recv_s;
```

### 2. 底盘命令 (cmd_controller.h)
```c
typedef struct {
    float vx;          // 平滑后的前进速度
    float vy;          // 平滑后的侧向速度
    float wz;          // 平滑后的旋转速度
    bool enabled;      // 是否启用
} ChassisCmd;
```

## 关键处理步骤

### Step 1: RadarComm接收和解析
**文件**: `modules/radar_comm/radar_comm.c`

```c
// 接收原始USB数据
void RadarComm_RxCallback(uint8_t *buf, uint32_t len)
{
    // 写入Ring Buffer (ISR-safe)
    ring_buffer_write(buf, len);
}

// 主task中处理
void RadarComm_Task(void)
{
    // 1. 查找同步字节: 0xA5 0x5A
    // 2. 读取12字节数据 (3个float)
    // 3. 读取CRC8校验码
    // 4. 计算CRC并比对
    // 5. 提取vx/vy/wz (little-endian float)
    // 6. 发布TOPIC_RADAR_CMD
}
```

**特点**:
- ✅ Ring Buffer处理USB分片
- ✅ Sync查找恢复数据损坏
- ✅ CRC8防止EMI错误
- ✅ 200ms超时检测

**可能的问题**:
- ❌ CRC校验失败 → 数据丢弃
- ❌ 无同步字节 → 等待下一个
- ❌ 超时500ms → 标记invalid

### Step 2: CmdController接收Radar消息
**文件**: `application/cmd/cmd_controller.c`

```c
// 消息回调 (快速path)
static void on_radar_update(const MsgEvent *ev, void *data)
{
    if (ev && ev->data) {
        // 保存最新的radar数据
        memcpy(&s_last_radar, ev->data, sizeof(Radar_Recv_s));
    }
}

// 主循环 (200Hz)
void CmdController_Task(uint32_t current_tick)
{
    uint32_t now = HAL_GetTick();
    
    // 优先级1: Radar模式 (如果数据有效且新鲜)
    if (s_last_radar.valid && (now - s_last_radar.ts_ms <= 500u)) {
        radar_cmd_to_wheel_speeds(s_last_radar.vx, s_last_radar.vy, s_last_radar.wz, now);
    }
    // 优先级2: RC模式 (后备)
    else {
        process_chassis_command(&s_last_rc, ...);
    }
}
```

**超时保护**:
```
如果500ms没收到新的radar数据 → 自动切回RC模式
                          ↓
                    安全止损机制
```

### Step 3: 平滑过滤处理
**文件**: `application/cmd/cmd_controller.c` (第255-290行)

```c
// 参数 (match Python脚本)
#define RADAR_SMOOTH_ALPHA 0.20f    // 低通滤波系数
#define RADAR_MAX_DELTA_V 0.05f     // 最大加速度限制 (m/s per cycle)
#define RADAR_MAX_DELTA_W 0.10f     // 最大角加速度限制 (rad/s)

// 处理流程
float prev_vx = s_filtered_vx;

// 1. 低通滤波 (平滑)
float lp_vx = prev_vx + RADAR_SMOOTH_ALPHA * (vx - prev_vx);

// 2. 加速度限制 (防止急转)
float dvx = lp_vx - prev_vx;
if (dvx > RADAR_MAX_DELTA_V) dvx = RADAR_MAX_DELTA_V;
if (dvx < -RADAR_MAX_DELTA_V) dvx = -RADAR_MAX_DELTA_V;

s_filtered_vx = prev_vx + dvx;
```

**执行频率**: 200Hz (REFRESH_HZ)

### Step 4: 坐标变换 (舵轮专用)
**文件**: `application/cmd/cmd_controller.c` (第104-250行)

```c
// 将(vx, vy, wz) → (舵轮角度, 驱动转速)
static void radar_cmd_to_wheel_speeds(float vx, float vy, float wz, uint32_t now)
{
    // 1. 基于舵轮几何计算轮速
    //    Module 0 (前): v0_x, v0_y → theta0, speed0
    //    Module 1 (后): v1_x, v1_y → theta1, speed1
    
    // 2. 转换为encoder ticks
    float tick0 = theta0 * rad_to_ticks;
    float tick1 = theta1 * rad_to_ticks;
    
    // 3. 最短路径优化 (舵轮转向不超过90°)
    // 4. 应用initial offset
    
    // 5. 发布到ChassisController
    ChassisController_SetSteerTargetAngles(ctrl, steer_angles);
    ChassisController_SetTargetSpeeds(ctrl, drive_speeds);
}
```

### Step 5: ChassisController执行
**文件**: `application/chassis/sentry_controller.c`

```c
// 接收转向角度目标
void ChassisController_SetSteerTargetAngles(ChassisController *controller, const float angles[2])
{
    controller->steer_target_angles[0] = angles[0];
    controller->steer_target_angles[1] = angles[1];
}

// 接收驱动速度目标
void ChassisController_SetTargetSpeeds(ChassisController *controller, const float speeds[4])
{
    for (int i = 0; i < 4; i++) {
        controller->target_speeds[i] = speeds[i];
    }
}

// 计算PID输出
void ChassisController_ComputeCurrents(ChassisController *controller, uint32_t current_tick)
{
    // 转向: GM6020角度PID
    // 驱动: M3508速度PID
}
```

---

## 数据诊断

### 检查点1: Radar是否接收到数据?
查看日志:
```
[STM32_LOG] RadarComm: vx=0.500 vy=0.200 wz=0.000 frames=123 errors=0
```

**如果没有**:
- ❌ Python脚本CRC计算错误
- ❌ 串口连接不稳定 (EMI)
- ❌ USB CDC未正常初始化

### 检查点2: Radar数据是否进入CmdController?
查看日志模式 (10Hz输出):
```
CMD,RADAR,0.500,0.200,0.000
```

**如果是RADAR模式但没数据**:
- ❌ MessageCenter没有转发TOPIC_RADAR_CMD
- ❌ on_radar_update没被调用

### 检查点3: 数据是否被平滑了?
对比原始值 vs 平滑值:
```
RADAR,IN:0.500,0.200,0.000,OUT:0.498,0.198,0.000
```
(输出应该略小于输入)

**如果没有平滑**:
- ❌ 平滑参数没生效
- ❌ radar_cmd_to_wheel_speeds没被调用

### 检查点4: 舵轮是否收到目标?
查看电机目标值:
```
STEER_TARGET[0]=1234 STEER_TARGET[1]=5678
```

**如果没有**:
- ❌ 坐标变换计算错误
- ❌ ChassisController_SetSteerTargetAngles没被调用

---

## radar_comm可能的改进点

### 现有设计 ✅ 
- ✅ Ring Buffer处理USB分片
- ✅ CRC8校验防错
- ✅ Sync自恢复

### 可能的改进 (如果有问题)

#### 1. 数据验证不足
```c
// 添加
if (vx != vx) return;  // NaN检查
if (fabs(vx) > 10.0f) return;  // 范围检查
```

#### 2. 时间戳精度
```c
// 现在: HAL_GetTick() (1ms精度)
// 可改: 使用DWT_CYCCNT (微秒精度)
```

#### 3. 多包重组
```c
// 如果USB分多次发送, 可能第一包不完整
// 现在通过Ring Buffer + Sync自动恢复
// 可改: 显式等待完整15字节后再处理
```

#### 4. 统计日志
```c
// 添加更多诊断:
- 帧间隔时间分布
- CRC失败原因 (同步错误 vs 数据损坏)
- 平滑前后差值
```

---

## 总结

**数据流完整, 不是radar_comm的问题, 很可能是**:

1. **Python发送不稳定** → 用新的`cmd_vel_keyboard_fixed.py` ✅ 已修复
2. **STM32处理频率不够** → 提升Python发送频率到100Hz ✅ 已修复
3. **缺少速度平滑** → 加入低通滤波 ✅ 已修复
4. **超时保护过严** → 检查500ms超时是否太短

**建议**: 先用修复后的脚本测试, 如果还有问题再检查CRC或EMI。
