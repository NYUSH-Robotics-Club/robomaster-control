# 代码模块分布（步兵 vs 哨兵）

## 总体架构

```
固件结构分层：
├─ Modules（公共库）
│  ├─ radar_comm ✅ 通用（接收 USB CDC 数据）
│  ├─ vision_comm ✅ 通用
│  ├─ message_center ✅ 通用
│  ├─ logger ✅ 通用
│  └─ ...
├─ Application（应用层）
│  ├─ cmd_controller.c ⚠️  公共但含条件编译
│  ├─ shoot_controller.c ✅ 公共
│  ├─ gimbal_controller.c ✅ 公共
│  └─ chassis/
│     ├─ chassis_controller.c (步兵)
│     └─ sentry_controller.c (哨兵)
```

---

## 详细分类表

### 1. 步兵专用（Infantry Standard）

| 文件 | 包含内容 |
|------|---------|
| `application/chassis/chassis_controller.c` | 4-wheel mecanum 控制逻辑 |
| `config/infantry_standard.h` | 步兵电机配置（3508x4 驱动 + 云台） |

**编译指令**：
```bash
cmake -S . -B build -DROBOT_TYPE=infantry_standard
```

---

### 2. 哨兵专用（Sentry Swerve）

| 文件 | 包含内容 |
|------|---------|
| `application/chassis/sentry_controller.c` | 2-module swerve 控制逻辑 |
| `config/sentry_swerve.h` | 哨兵电机配置（3508x4 驱动 + GM6020x2 舵机） |

**编译指令**：
```bash
cmake -S . -B build -DROBOT_TYPE=sentry_swerve
```

---

### 3. 公共代码（两个机型共享）

#### ✅ 完全通用

| 文件 | 原因 |
|------|------|
| `modules/radar_comm/radar_comm.h/.c` | 通用 USB CDC 协议处理 |
| `modules/vision_comm/vision_comm.h/.c` | 通用视觉通信 |
| `modules/logger/logger.h/.c` | 通用日志 |
| `modules/message_center/message_center.h/.c` | 通用消息总线 |
| `modules/imu/` | 通用 IMU 驱动 |
| `application/gimbal_controller.c` | 云台控制（两者都有） |
| `application/shoot_controller.c` | 发射控制（两者都有） |
| `Src/main.c` | 主循环（条件编译分支处理） |

#### ⚠️ 条件编译分支（`cmd_controller.c`）

**公共部分**：
```c
#include "radar_comm.h"

// 所有回调通用
static void on_rc_update() { ... }
static void on_imu_update() { ... }
static void on_vision_update() { ... }
static void on_radar_update() { ... }

// 通用帮助函数
static void gimbal_to_chassis_frame() { ... }
static void process_shooter_command() { ... }
static void process_gimbal_command() { ... }
static void process_chassis_command() { ... }
```

**条件编译分支**：
```c
#ifdef ROBOT_TYPE_sentry_swerve
    // 哨兵特定：swerve 映射 + 优先级控制
    static void radar_cmd_to_wheel_speeds() { ... }
    // 在 CmdController_Task() 中:
    if (radar_valid) {
        radar_cmd_to_wheel_speeds(...);  // 哨兵才支持
    } else {
        process_chassis_command(...);    // 通用 RC 控制
    }
#else  // Infantry
    // 步兵特定：传递模式（placeholder）
    static void radar_cmd_to_wheel_speeds() { ... }
    // 在 CmdController_Task() 中:
    // 仅 RC 控制，radar 支持 TBD
    process_chassis_command(...);
#endif
```

---

## 编译流程

### 步兵编译
```bash
cmake -S . -B build -DROBOT_TYPE=infantry_standard

# 实际编译的文件：
# ✅ modules/radar_comm/radar_comm.c
#    ├─ on_radar_update() 被 cmd_controller.c 订阅
#    └─ 但雷达数据不用（placeholder 模式）
# ✅ application/cmd/cmd_controller.c (else 分支)
# ✅ application/chassis/chassis_controller.c
# ❌ application/chassis/sentry_controller.c (不编译)
```

### 哨兵编译
```bash
cmake -S . -B build -DROBOT_TYPE=sentry_swerve

# 实际编译的文件：
# ✅ modules/radar_comm/radar_comm.c
#    ├─ on_radar_update() 被 cmd_controller.c 订阅
#    └─ 雷达数据被完全使用（swerve 映射）
# ✅ application/cmd/cmd_controller.c (#ifdef 分支)
# ❌ application/chassis/chassis_controller.c (不编译)
# ✅ application/chassis/sentry_controller.c
```

---

## 关键代码段（机型判断）

### 1. cmd_controller.c 中的条件编译

```c
// 雷达映射函数 - 机型特定
#ifdef ROBOT_TYPE_sentry_swerve
    static void radar_cmd_to_wheel_speeds(...)  // Swerve 映射
#else
    static void radar_cmd_to_wheel_speeds(...)  // 直接传递（Infantry）
#endif

// 主任务 - 机型特定的模式选择
void CmdController_Task(...) {
    #ifdef ROBOT_TYPE_sentry_swerve
        if (radar_valid) {
            radar_cmd_to_wheel_speeds(...);    // 哨兵支持雷达
        } else {
            // RC 控制...
        }
    #else
        // Infantry: RC 控制（目前不支持雷达优先级）
        process_chassis_command(...);
    #endif
}
```

### 2. sentry_swerve.h（电机配置）

```c
// 哨兵特定：2 个舵机
{
    .motor_id = 5, .type = MOTOR_TYPE_GM6020,
    .role = MOTOR_ROLE_CHASSIS_STEER, ...
},
{
    .motor_id = 6, .type = MOTOR_TYPE_GM6020,
    .role = MOTOR_ROLE_CHASSIS_STEER, ...
}
```

### 3. infantry_standard.h（电机配置）

```c
// 步兵特定：4 个 mecanum 轮（无舵机）
{
    .motor_id = 0, .type = MOTOR_TYPE_M3508,
    .role = MOTOR_ROLE_CHASSIS_DRIVE, ...
},
{
    .motor_id = 1, .type = MOTOR_TYPE_M3508,
    .role = MOTOR_ROLE_CHASSIS_DRIVE, ...
},
// ... 只有驱动电机，没有转向
```

---

## 功能对照表

| 功能 | 步兵 | 哨兵 | 说明 |
|------|------|------|------|
| RC 遥控 | ✅ | ✅ | 公共实现 |
| 视觉接收 | ✅ | ✅ | 公共实现 |
| **雷达自主** | ❌ | ✅ | 哨兵仅有（通过 swerve 映射） |
| **Swerve 控制** | ❌ | ✅ | 哨兵独有（2 舵机） |
| **Mecanum 控制** | ✅ | ❌ | 步兵独有（4 轮） |
| 云台控制 | ✅ | ✅ | 公共实现 |
| 发射控制 | ✅ | ✅ | 公共实现 |

---

## 如何添加步兵雷达支持（可选）

若要在步兵上也启用雷达自主，只需修改：

**在 `cmd_controller.c` 的 infantry 分支中添加**：
```c
#else  // Infantry
    if (s_last_radar.valid && (now - s_last_radar.ts_ms <= 500u)) {
        // Infantry: use radar command (direct pass-through to mecanum controller)
        radar_cmd_to_wheel_speeds(...);
        process_shooter_command(&s_last_rc);
        process_gimbal_command(&s_last_rc, &s_last_sensor, false);
    } else {
        // RC mode
        process_chassis_command(...);
    }
#endif
```

然后步兵也能用雷达控制（但不涉及舵机转向，只是 vx/vy/wz 直接给 mecanum 控制器）。

---

## 总结

| 层级 | 分布 |
|------|------|
| **库层（modules）** | 100% 通用（包括 radar_comm） |
| **应用层（application）** | 公共 + 机型条件分支 |
| **配置层（config）** | 机型特定（电机表） |

✅ **现在步兵编译时不会包含哨兵特定的 swerve 逻辑**（通过 `#ifdef` 隔离）
✅ **雷达模块在两者上都会编译，但只有哨兵会优先使用**
✅ **完全向后兼容**（步兵行为不变）
