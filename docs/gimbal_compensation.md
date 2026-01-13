# 云台倾斜角度补偿说明

## 概述

本功能实现了云台倾斜时的角度补偿计算和显示，用于补偿pitch轴倾斜时yaw旋转产生的耦合效应。

## 物理模型

### 问题描述
- 云台高度：30cm（距地面）
- 当云台pitch轴有倾角θ_p时，yaw轴旋转会产生pitch方向的视觉偏移
- 这种耦合效应需要实时补偿以保持准确的目标锁定

### 数学模型

#### 1. Yaw旋转导致的Pitch耦合
当pitch角度为θ_p，yaw旋转Δθ_y时：

```
Δθ_pitch = sin(Δθ_y) × tan(θ_p)
```

这个补偿量表示：当yaw旋转时，由于pitch倾斜，枪口指向在pitch方向的偏移量。

#### 2. 目标水平距离计算
假设目标在地面上，云台高度为h = 30cm：

```
d = h / tan(θ_p)
```

其中：
- d：目标水平距离（cm）
- h：云台高度（30cm）
- θ_p：pitch角度（弧度）

#### 3. Pitch变化时的Yaw补偿系数
当pitch角度改变时，为保持指向同一目标：

```
补偿系数 = sin(θ_p) / cos(θ_p) = tan(θ_p)
```

## 实现细节

### 文件修改
- `application/gimbal/gimbal_controller.c`
- `application/gimbal/gimbal_controller.h`

### 核心功能

#### Pitch轴控制策略（自瞄模式）
当视觉系统激活（`vision_valid = true`）时：
- ✅ 视觉系统不控制pitch轴（只控制yaw）
- ✅ 遥控器可以手动调整pitch角度
- ✅ 禁用yaw-pitch耦合补偿（避免干扰yaw追踪）
- ✅ 保持重力补偿以稳定pitch位置
- ✅ 适用于需要人工微调pitch的场景

**设计理念**：
- 视觉系统专注于水平追踪（yaw）
- 操作员根据目标高度手动调整pitch
- 实现人机协同控制

### 核心函数

#### 1. `GimbalController_CalculateAndDisplayCompensation()`
**功能**：
- 计算当前pitch和yaw角度
- 计算目标水平距离
- 计算yaw-pitch耦合补偿值
- 显示补偿信息（USB CDC）
- 记录到CSV日志

**更新频率**：每100ms更新一次

**输出格式**：
```
=== 云台角度补偿计算 ===
云台高度: 30.0 cm
当前Pitch角度: XX.XX° (X.XXXX rad)
当前Yaw角度: XX.XX°
Pitch控制: 遥控器手动 (自瞄时视觉不控制pitch)
目标水平距离: XXX.X cm

补偿值:
- Yaw旋转1°时的Pitch耦合: X.XXXX° (X.XXXXXX rad)
- Pitch变化1°时需要的Yaw补偿系数: X.XXXX
- Yaw旋转1°的Pitch补偿(编码器刻度): XX.XX ticks
========================
```

**模式说明**：
- "Pitch控制: 遥控器手动 (自瞄时视觉不控制pitch)" - 自瞄模式
- "Pitch控制: 遥控器手动 + Yaw-Pitch耦合补偿" - 手动模式

**CSV日志格式**：
```
GIM,timestamp,COMPENSATION,pitch_deg,yaw_deg,distance_cm,pitch_comp_deg,yaw_comp_coeff,pitch_comp_ticks
```

#### 2. Pitch控制（在`GimbalController_PitchControl`中）

**函数原型**：
```c
int16_t GimbalController_PitchControl(uint8_t id, float rate_normalized,
                                      SensorData *sensor_data,
                                      bool disable_yaw_pitch_compensation);
```

**控制逻辑**：

##### A. 遥控器输入（始终有效）
```c
// 始终响应遥控器输入
float sensitivity = 60.0f;
c->angle_target += c->config->direction * sensitivity * rate_normalized;
```
- 无论自瞄或手动模式，遥控器都可以控制pitch
- 操作员可以随时调整pitch角度

##### B. Yaw-Pitch耦合补偿（可选）
```c
// 只在手动模式下启用
if (!disable_yaw_pitch_compensation && yaw && yaw->angle_initialized) {
    // 计算yaw角度变化
    float yaw_delta = current_yaw_angle - last_yaw_angle;

    // 计算pitch补偿
    float pitch_compensation_rad = sin(yaw_delta_rad) * tan(pitch_angle_rad);

    // 反向应用补偿以抵消耦合效应
    pitch_target -= pitch_compensation_ticks;
}
```

**两种模式对比**：

| 特性 | 自瞄模式 | 手动模式 |
|-----|---------|---------|
| **遥控器输入** | ✅ 有效 | ✅ 有效 |
| **Yaw-Pitch补偿** | ❌ 禁用 | ✅ 启用 |
| **重力补偿** | ✅ 启用 | ✅ 启用 |
| **适用场景** | Yaw自动追踪，Pitch手动 | 完全手动控制 |

**模式切换**：
```c
int16_t pitch_current = GimbalController_PitchControl(
    s_pitch_motor_id,
    s_last_cmd.pitch_rate,         // ← 始终传递遥控器输入
    &s_last_sensor,
    use_vision_target  // ← true=禁用补偿, false=启用补偿
);
```

## 使用方法

### 1. 编译固件
```bash
cd /home/nyu/Codespace/robomaster-control
cmake --build build
```

### 2. 烧录到板子
```bash
arm-none-eabi-objcopy -O binary build/NYUSH_Infantry.elf build/NYUSH_Infantry.bin
dfu-util -a 0 -s 0x08000000:leave -D build/NYUSH_Infantry.bin
```

### 3. 查看补偿信息

#### 方法1：USB CDC输出
使用串口终端连接到板子的USB虚拟串口，补偿信息每100ms更新一次。

```bash
# Linux
screen /dev/ttyACM1 115200

# 或使用minicom
minicom -D /dev/ttyACM1
```

#### 方法2：CSV日志分析
使用smart_logger.py工具实时监控：

```bash
python3 script/smart_logger.py --tags GIM
```

日志会包含COMPENSATION类型的数据，格式为：
```
GIM,timestamp,COMPENSATION,pitch_deg,yaw_deg,distance_cm,pitch_comp_deg,yaw_comp_coeff,pitch_comp_ticks
```

### 4. 测试补偿效果

#### A. Pitch遥控器控制测试（自瞄模式）

1. **基本响应测试**：
   - 将pitch轴调整到某个角度（如15°）
   - 触发自瞄模式（`vision_valid = true`）
   - **关键**：用遥控器控制pitch，验证可以响应
   - 观察USB CDC输出显示 "Pitch控制: 遥控器手动 (自瞄时视觉不控制pitch)"

2. **Yaw追踪时的Pitch独立控制**：
   - 启动自瞄模式，yaw开始追踪目标
   - 用遥控器调整pitch角度
   - 验证yaw继续追踪，不受pitch调整影响
   - 确认pitch调整不会干扰yaw追踪

3. **模式切换测试**：
   - 在自瞄模式和手动模式之间切换
   - 验证pitch在两种模式下都响应遥控器
   - 检查切换时pitch控制是否平滑

#### B. 静态补偿计算测试

1. **补偿值验证**：
   - 将pitch轴调整到不同角度（如10°、20°、30°）
   - 观察补偿值的变化
   - 验证目标水平距离计算是否合理

2. **理论对比**：
   - 记录pitch=15°时的补偿值
   - 手动计算：`compensation = sin(1°) × tan(15°) ≈ 0.00467 rad ≈ 0.267°`
   - 对比固件输出值

#### C. 动态补偿测试（手动模式）

1. **Yaw-Pitch耦合补偿**：
   - 固定pitch角度（如15°）
   - 旋转yaw轴
   - 观察pitch是否保持稳定（补偿生效）
   - 记录pitch角度变化量（应<1°）

2. **目标锁定测试**：
   - 设置一个固定地面目标（如1米外）
   - 改变pitch角度
   - 验证yaw角度是否自动调整以保持目标锁定

## 补偿参数

### 可调参数
在`gimbal_controller.c`中定义：

```c
#define GIMBAL_HEIGHT_CM (30.0f)              // 云台高度（可根据实际调整）
#define COMPENSATION_UPDATE_RATE_MS (100)     // 显示更新频率
```

### 补偿阈值
在pitch控制中，只有当yaw变化超过1.0刻度时才应用补偿：

```c
if (fabsf(yaw_delta) > 1.0f && (current_time - last_yaw_time) > 0) {
    // 计算并应用补偿
}
```

## 理论验证

### 示例计算
假设：
- pitch角度：15°
- yaw旋转：10°
- 云台高度：30cm

**计算过程**：
1. pitch = 15° = 0.2618 rad
2. yaw_delta = 10° = 0.1745 rad
3. pitch_compensation = sin(0.1745) × tan(0.2618)
                      = 0.1736 × 0.2679
                      = 0.0465 rad
                      = 2.66°

**物理意义**：
当pitch为15°时，yaw旋转10°会导致枪口指向在pitch方向偏移约2.66°。通过实时补偿这个偏移，可以保持目标锁定精度。

### 距离计算验证
- pitch = 15°，height = 30cm
- distance = 30 / tan(15°) = 30 / 0.2679 = 112cm

这意味着当pitch为15°时，云台指向的是距离约1.12米的地面目标。

## 注意事项

1. **补偿方向**：
   - 补偿量是**反向**应用的（`pitch_target -= compensation`）
   - 这是为了抵消耦合效应，而不是增强它

2. **角度环绕处理**：
   - yaw角度在0-8192范围内环绕
   - 代码中包含环绕检测和处理逻辑

3. **除零保护**：
   - 当pitch接近0°时，避免除零错误
   - 使用阈值检测（`fabsf(pitch) > 0.01f`）

4. **更新频率**：
   - 显示更新：100ms
   - 补偿应用：每个控制周期（~5ms）

5. **性能影响**：
   - 三角函数计算（sin, tan）在每次调用时执行
   - 对控制周期的影响可以忽略不计（<1%）

## 使用场景

### 自瞄模式（视觉控制Yaw，遥控器控制Pitch）
**适用场景**：
- 地面目标追踪（装甲板识别）
- 不同高度/距离的目标切换
- 需要快速反应的战斗场景

**操作流程**：
1. 手动粗调pitch到大致角度
2. 触发自瞄，yaw自动追踪目标
3. 根据目标高度变化，操作员微调pitch
4. 实现人机协同控制

**优势**：
- **灵活性**：操作员可随时调整pitch
- **稳定性**：Yaw追踪不受pitch调整干扰
- **简化视觉**：视觉系统只处理水平追踪
- **人机协同**：结合视觉快速追踪和人工判断

**典型场景**：
- 追踪移动目标时，目标上下坡或台阶
- 不同距离目标切换（近→远需要调整pitch）
- 多层目标切换（高台↔地面）

### 手动模式（完全遥控器控制）
**适用场景**：
- 手动瞄准
- 探测和搜索目标
- 复杂战术机动

**优势**：
- 完全灵活控制
- Yaw-pitch耦合自动补偿
- 适应各种战术需求

## 未来改进

### 已实现
- ✅ 自瞄模式下Pitch遥控器控制（不受视觉控制）
- ✅ Yaw-Pitch耦合补偿（手动模式）
- ✅ 自瞄时禁用yaw-pitch补偿（避免干扰）
- ✅ 实时补偿值显示和模式状态
- ✅ CSV日志记录

### 计划改进

1. **自适应云台高度**：
   - 从配置文件读取云台高度
   - 支持不同机器人配置

2. **补偿强度调节**：
   - 添加补偿增益系数
   - 允许用户调节补偿强度

3. **视觉Pitch控制（可选）**：
   - 在自瞄模式下，可选择让视觉控制pitch
   - 通过配置参数切换pitch锁定/视觉控制
   - 适用于高度变化的目标

4. **IMU融合**：
   - 结合IMU数据进行更精确的补偿
   - 考虑底盘运动对补偿的影响

5. **滤波优化**：
   - 对yaw速度进行低通滤波
   - 减少高频抖动对补偿的影响

## 参考资料

- [CLAUDE.md](../CLAUDE.md) - 项目架构说明
- [architecture.md](architecture.md) - 详细架构文档
- GM6020电机手册 - 编码器规格
- RoboMaster规则手册 - 机器人尺寸规范
