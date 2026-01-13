# Vision System Diagnostics and Parameter Tuning

## 概述 (Overview)

本文档说明下位机视觉通信系统的诊断和参数调整功能。这些功能用于解决云台不响应视觉系统的各种问题。

This document describes the diagnostics and parameter tuning features for the lower-level vision communication system. These features help diagnose and resolve issues where the gimbal doesn't respond to vision system commands.

## 目录 (Table of Contents)

1. [问题诊断 (Problem Diagnosis)](#问题诊断)
2. [数据验证 (Data Validation)](#数据验证)
3. [通信诊断 (Communication Diagnostics)](#通信诊断)
4. [参数调整 (Parameter Tuning)](#参数调整)
5. [命令接口 (Command Interface)](#命令接口)
6. [使用示例 (Usage Examples)](#使用示例)

---

## 问题诊断 (Problem Diagnosis)

### 云台不响应的常见原因 (Common Reasons for Gimbal Non-Response)

根据您提供的分析，即使检测到的角度偏差大于死区，云台仍可能不转动。系统现在可以检测以下问题：

#### 1. 控制算法问题 (Control Algorithm Issues)

**积分饱和 (Integral Saturation)**
- 症状：PID积分项累积过大
- 检测：`ControlDiagnostics.integral_saturated`
- 监控：`ControlDiagnostics.saturation_count`
- 解决：自动检测并记录，建议调整Ki参数

**输出饱和 (Output Saturation)**
- 症状：控制信号超出执行能力范围
- 检测：`ControlDiagnostics.output_saturated`
- 监控：检查`pid_output_pitch`和`pid_output_yaw`是否接近`CURRENT_LIMIT`
- 解决：调整PID参数或检查机械负载

**死区问题 (Deadzone Issues)**
- 症状：角度偏差在死区范围内
- 检测：`ControlDiagnostics.in_deadzone`
- 监控：`angle_error_pitch`和`angle_error_yaw`
- 解决：调整`converge_threshold`参数

#### 2. 数据处理问题 (Data Processing Issues)

**数据精度损失 (Data Precision Loss)**
- 症状：接收的数据与发送的数据有差异
- 检测：比较`VisionDataQuality.pitch_raw`与滤波后的值
- 监控：所有角度数据使用float类型，避免精度损失
- 解决：检查上位机数据格式

**数据滤波过度 (Over-Filtering)**
- 症状：有效的角度变化被滤除
- 检测：`VisionDataQuality.noise_estimate`过大
- 监控：比较raw和filtered数据的差异
- 解决：调整`filter_window`或`ema_alpha`参数

#### 3. 通信问题 (Communication Issues)

**通信延迟 (Communication Latency)**
- 症状：接收的数据"过时"
- 检测：`VisionDiagnostics.avg_latency_ms`和`max_latency_ms`
- 监控：`last_recv_time_ms`
- 解决：检查USB连接，优化上位机发送频率

**通信干扰 (Communication Interference)**
- 症状：数据传输错误
- 检测：`VisionDiagnostics.crc_errors`
- 监控：`length_errors`和`cmd_id_errors`
- 解决：检查USB线缆，避免电磁干扰源

**数据超时 (Data Timeout)**
- 症状：长时间未收到数据
- 检测：`VisionDiagnostics.is_data_stale`
- 监控：`timeout_count`
- 解决：检查上位机程序运行状态

#### 4. 数据有效性 (Data Validity)

**角度超出范围 (Out of Range)**
- 症状：接收的角度超出物理限制
- 检测：`ValidationResult.VALIDATION_OUT_OF_RANGE`
- 监控：`out_of_range_count`
- 解决：检查上位机计算是否正确

---

## 数据验证 (Data Validation)

### 验证项目 (Validation Items)

#### 1. 数据包完整性 (Packet Integrity)
```c
// 检查包长度
if (len < 18 || len > VISION_RECV_SIZE) {
    g_diag.length_errors++;
    return VALIDATION_LENGTH_ERROR;
}
```

#### 2. 命令ID验证 (Command ID Validation)
```c
if (cmd_id != 0x0001) {
    g_diag.cmd_id_errors++;
    return VALIDATION_CMD_ID_ERROR;
}
```

#### 3. 角度范围检查 (Angle Range Check)
```c
// Pitch: -45° to +45° (default)
// Yaw: -180° to +180° (default)
if (pitch < cfg->pitch_min || pitch > cfg->pitch_max ||
    yaw < cfg->yaw_min || yaw > cfg->yaw_max) {
    return VALIDATION_OUT_OF_RANGE;
}
```

#### 4. 超时检测 (Timeout Detection)
```c
uint32_t elapsed = current_time - g_diag.last_recv_time_ms;
if (elapsed > cfg->data_timeout_ms) {
    g_diag.timeout_count++;
    return VALIDATION_TIMEOUT;
}
```

### 启用/禁用验证 (Enable/Disable Validation)

```c
VisionConfig *cfg = VisionConfig_Get();
cfg->enable_data_validation = true;   // 启用验证
cfg->enable_timeout_check = true;     // 启用超时检查
```

---

## 通信诊断 (Communication Diagnostics)

### 诊断数据结构 (Diagnostics Structure)

```c
typedef struct {
    // 数据包统计
    uint32_t total_received;        // 总接收包数
    uint32_t total_sent;            // 总发送包数
    uint32_t crc_errors;            // CRC错误次数
    uint32_t timeout_count;         // 超时次数
    uint32_t out_of_range_count;    // 超出范围次数

    // 通信质量
    float packet_loss_rate;         // 丢包率 (0.0-1.0)
    uint32_t avg_latency_ms;        // 平均延迟
    uint32_t max_latency_ms;        // 最大延迟

    // 数据质量
    float data_noise_level;         // 噪声水平
    uint32_t last_recv_time_ms;     // 最后接收时间
    bool is_data_stale;             // 数据是否过时

    // 协议错误
    uint32_t length_errors;         // 长度错误次数
    uint32_t cmd_id_errors;         // 命令ID错误次数
} VisionDiagnostics;
```

### 获取诊断信息 (Get Diagnostics)

```c
// 获取诊断数据
VisionDiagnostics *diag = VisionComm_GetDiagnostics();

// 打印诊断报告到USB CDC
VisionComm_PrintDiagnostics();

// 重置诊断计数器
VisionComm_ResetDiagnostics();
```

### 数据质量监控 (Data Quality Monitoring)

```c
typedef struct {
    float pitch_filtered;           // 滤波后pitch
    float yaw_filtered;             // 滤波后yaw
    float pitch_raw;                // 原始pitch
    float yaw_raw;                  // 原始yaw
    float noise_estimate;           // 噪声估计
    uint32_t timestamp_ms;          // 接收时间戳
    ValidationResult validation;    // 验证结果
} VisionDataQuality;

// 获取数据质量信息
VisionDataQuality *quality = VisionComm_GetDataQuality();
```

### 控制信号诊断 (Control Signal Diagnostics)

```c
typedef struct {
    // PID输出监控
    float pid_output_pitch;         // Pitch PID输出
    float pid_output_yaw;           // Yaw PID输出
    float integral_pitch;           // Pitch积分项
    float integral_yaw;             // Yaw积分项

    // 饱和检测
    bool integral_saturated;        // 积分饱和标志
    bool output_saturated;          // 输出饱和标志
    uint32_t saturation_count;      // 饱和发生次数

    // 响应分析
    float angle_error_pitch;        // 当前pitch误差
    float angle_error_yaw;          // 当前yaw误差
    float deadzone_threshold;       // 当前死区阈值
    bool in_deadzone;               // 是否在死区内
} ControlDiagnostics;

// 更新控制诊断（在gimbal_controller中调用）
VisionComm_UpdateControlDiag(pitch_pid_out, yaw_pid_out,
                               pitch_error, yaw_error);

// 获取控制诊断
ControlDiagnostics *ctrl_diag = VisionComm_GetControlDiagnostics();
```

---

## 参数调整 (Parameter Tuning)

### 可调参数 (Adjustable Parameters)

#### 1. 滤波参数 (Filter Parameters)

**滑动平均滤波窗口 (Moving Average Window)**
```c
cfg->filter_window = 5;  // 1-10, 默认5
// 值越大 = 更平滑但延迟更大
// 值越小 = 响应更快但噪声更大
```

**指数移动平均系数 (EMA Alpha)**
```c
cfg->ema_alpha = 0.3f;  // 0.0-1.0, 默认0.3
// 值越大 = 更重视新数据（响应快）
// 值越小 = 更重视历史数据（更平滑）
```

#### 2. 数据融合参数 (Data Fusion Parameters)

**视觉数据权重 (Vision Data Weight)**
```c
cfg->fusion_weight = 0.7f;  // 0.0-1.0, 默认0.7
// 1.0 = 完全信任视觉数据
// 0.0 = 完全忽略视觉数据（仅使用IMU）
```

**收敛阈值 (Convergence Threshold)**
```c
cfg->converge_threshold = 0.05f;  // 弧度, 约3度
// 误差小于此值时认为已收敛
```

#### 3. 通信参数 (Communication Parameters)

**发送频率 (Send Interval)**
```c
cfg->send_interval_ms = 10;  // 5-50ms, 默认10ms (100Hz)
// 值越小 = 频率越高，实时性越好，但CPU负担越大
```

**数据超时 (Data Timeout)**
```c
cfg->data_timeout_ms = 200;  // 50-1000ms, 默认200ms
// 超过此时间未收到数据则认为数据过时
```

#### 4. 角度范围 (Angle Limits)

```c
cfg->pitch_min = -0.785398f;  // -45°
cfg->pitch_max = 0.785398f;   //  45°
cfg->yaw_min = -3.14159f;     // -180°
cfg->yaw_max = 3.14159f;      //  180°
```

### 配置管理 (Configuration Management)

```c
// 获取当前配置
VisionConfig *cfg = VisionConfig_Get();

// 修改参数
cfg->filter_window = 7;
cfg->ema_alpha = 0.5f;

// 重置为默认值
VisionConfig_Reset();

// 保存/加载配置（未来功能）
VisionConfig_Save();
VisionConfig_Load();
```

---

## 命令接口 (Command Interface)

### 命令格式 (Command Format)

#### 二进制命令 (Binary Command)
```c
typedef struct {
    uint8_t header;      // 固定 0xAA
    uint8_t cmd;         // 命令ID
    float param;         // 命令参数
    uint8_t checksum;    // 校验和 (XOR)
} VisionCmdPacket;
```

#### 文本命令 (Text Command)
```
命令格式: <command_name> [parameter]
示例: set_filter_window 5
```

### 可用命令 (Available Commands)

#### 1. 诊断命令 (Diagnostics Commands)

```bash
get_diag                  # 打印诊断报告
reset_diag                # 重置诊断计数器
```

#### 2. 滤波参数 (Filter Parameters)

```bash
set_filter_window 5       # 设置滤波窗口 (1-10)
set_ema_alpha 0.3         # 设置EMA系数 (0.0-1.0)
toggle_filtering          # 开关滤波功能
```

#### 3. 数据融合 (Data Fusion)

```bash
set_fusion_weight 0.7     # 设置融合权重 (0.0-1.0)
```

#### 4. 通信参数 (Communication)

```bash
set_send_interval 10      # 设置发送间隔 (5-50ms)
set_timeout_ms 200        # 设置超时时间 (50-1000ms)
toggle_timeout            # 开关超时检查
```

#### 5. 验证参数 (Validation)

```bash
toggle_validation         # 开关数据验证
```

#### 6. 配置管理 (Configuration)

```bash
reset_config              # 重置所有参数为默认值
help                      # 显示帮助信息
```

### 发送命令 (Sending Commands)

#### 通过USB CDC发送文本命令

1. 使用串口工具连接到设备（例如：screen, minicom, PuTTY）
2. 发送命令文本，例如：`set_filter_window 5`
3. 查看响应：`[VCMD] Cmd=0x10 OK`

#### 通过Python脚本发送

```python
import serial

ser = serial.Serial('/dev/ttyACM1', 115200, timeout=1)

# 发送命令
ser.write(b'set_filter_window 5\n')

# 读取响应
response = ser.readline()
print(response.decode())

ser.close()
```

---

## 使用示例 (Usage Examples)

### 示例1：诊断云台不响应问题

```c
// 1. 获取并打印诊断信息
VisionComm_PrintDiagnostics();

// 输出示例：
// === Vision Communication Diagnostics ===
// Packets: RX=1523 TX=1500
// Errors: CRC=0 Timeout=3 Range=0
// Quality: Loss=1.5% Noise=0.0023 Stale=0
// Control: InDZ=1 Sat=0 (count=0)
// Angles: P=0.123/0.120 Y=0.456/0.450

// 2. 分析问题
VisionDataQuality *quality = VisionComm_GetDataQuality();
ControlDiagnostics *ctrl_diag = VisionComm_GetControlDiagnostics();

if (ctrl_diag->in_deadzone) {
    // 云台在死区内，这是正常的
    printf("Gimbal converged (in deadzone)\n");
}

if (ctrl_diag->output_saturated) {
    // 控制输出饱和，可能需要降低PID增益或检查负载
    printf("WARNING: Control output saturated!\n");
}

if (quality->validation != VALIDATION_OK) {
    // 数据验证失败
    printf("Data validation failed: %d\n", quality->validation);
}
```

### 示例2：调整滤波参数减少噪声

```bash
# 如果噪声太大，增加滤波窗口
set_filter_window 8

# 或者降低EMA alpha值（更重视历史数据）
set_ema_alpha 0.2

# 查看效果
get_diag
```

### 示例3：调整通信参数提高实时性

```bash
# 减少发送间隔，提高频率
set_send_interval 5     # 200Hz

# 但要注意CPU负担和USB带宽
```

### 示例4：处理通信延迟问题

```c
VisionDiagnostics *diag = VisionComm_GetDiagnostics();

if (diag->avg_latency_ms > 50) {
    // 延迟过大，检查原因
    if (diag->timeout_count > 10) {
        // 可能是上位机处理太慢
        printf("WARNING: High latency and timeouts detected\n");
    }
}

// 调整超时阈值
// set_timeout_ms 300  // 增加到300ms
```

### 示例5：检测积分饱和

```c
ControlDiagnostics *ctrl_diag = VisionComm_GetControlDiagnostics();

if (ctrl_diag->saturation_count > 100) {
    // 频繁饱和，建议调整PID参数
    printf("WARNING: Frequent saturation detected (%lu times)\n",
           ctrl_diag->saturation_count);
    printf("Consider adjusting PID parameters\n");
}
```

---

## 日志输出 (Logging Output)

### CSV格式数据 (CSV Data Format)

系统会自动输出CSV格式的日志数据，可以使用`script/smart_logger.py`工具查看：

```bash
python3 script/smart_logger.py --tags VIS
```

#### 接收数据日志 (RX Log)
```
VIS,timestamp,RX,pitch_raw,yaw_raw,pitch_filtered,yaw_filtered,target_state,validation
VIS,123456,RX,0.123,0.456,0.120,0.450,1,0
```

#### 控制诊断日志 (CTRL Log)
```
VIS,timestamp,CTRL,pitch_error,yaw_error,pitch_pid_out,yaw_pid_out,in_deadzone,saturated
VIS,123500,CTRL,0.023,0.034,1500.0,2000.0,0,0
```

---

## 故障排查流程 (Troubleshooting Workflow)

### 步骤1：检查通信状态
```bash
get_diag
```
检查：
- `total_received` > 0 （有接收数据）
- `crc_errors` 和 `timeout_count` 是否很大
- `is_data_stale` 是否为true

### 步骤2：检查数据质量
```c
VisionDataQuality *quality = VisionComm_GetDataQuality();
// 检查validation结果
// 检查noise_estimate是否过大
// 比较raw和filtered数据差异
```

### 步骤3：检查控制状态
```c
ControlDiagnostics *ctrl_diag = VisionComm_GetControlDiagnostics();
// 检查in_deadzone状态
// 检查output_saturated状态
// 检查angle_error大小
```

### 步骤4：参数调整
根据检查结果调整相应参数：
- 噪声大 → 增加滤波
- 延迟大 → 减小滤波窗口
- 饱和 → 调整PID参数
- 超时 → 检查上位机或增加超时阈值

---

## 总结 (Summary)

本诊断系统提供了全面的监控和调试能力，可以帮助定位和解决云台不响应的各种问题：

1. **数据验证**：确保接收的数据完整和合理
2. **通信诊断**：监控通信质量和错误
3. **滤波控制**：可调节的数据滤波以平衡噪声和延迟
4. **控制监控**：检测PID饱和和死区问题
5. **参数调整**：实时调整所有关键参数
6. **命令接口**：方便的文本命令调试接口

通过这些工具，可以快速诊断问题并优化系统性能。
