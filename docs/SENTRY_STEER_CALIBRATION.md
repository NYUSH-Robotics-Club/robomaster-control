# Sentry Swerve 转向电机校准指南

## 📋 问题说明

Sentry Swerve底盘有**4个驱动轮**和**2个转向电机**。启动时，转向电机会自动对齐到配置的`initial_angle`位置。如果`initial_angle`值不正确，轮子可能不会指向同一个方向。

## 🔍 当前配置

在 `config/sentry_swerve.h` 中：

```c
// Steer motor 5 (第一个转向电机)
.initial_angle = 1084.0f  // 需要根据实际硬件校准

// Steer motor 6 (第二个转向电机)  
.initial_angle = 2434.0f  // 需要根据实际硬件校准
```

## 🎯 校准目标

校准后，所有轮子应该：
- **指向同一个方向**（通常是"前进"方向）
- **在物理上对齐**（轮子平行）
- **编码器角度与物理位置匹配**

## 🔧 校准步骤

### 方法1：手动校准（推荐）

#### 步骤1：准备
1. 将底盘**架空**（轮子不接触地面）
2. 连接STM32和CAN总线
3. 确保所有电机已连接并上电

#### 步骤2：读取当前编码器值
1. 烧录代码（确保启用了日志）
2. 运行Python脚本连接USB CDC
3. 查看启动日志，找到转向电机的当前角度：

```
STM32> [DEBUG][SEN] Steer motor 5: target=1084.0 current=1234.0 error=150.0
STM32> [DEBUG][SEN] Steer motor 6: target=2434.0 current=2567.0 error=133.0
```

#### 步骤3：手动对齐轮子
1. **手动转动轮子**，使所有轮子指向同一个方向（前进方向）
2. **保持轮子不动**
3. 记录此时STM32输出的编码器值

#### 步骤4：更新配置
1. 打开 `config/sentry_swerve.h`
2. 找到转向电机的配置
3. 将 `initial_angle` 更新为步骤3记录的编码器值

```c
// 示例：如果motor 5当前编码器值是1200，motor 6是2500
{
    .motor_id = 5,
    // ... 其他配置 ...
    .limits.gm6020 = {
        .angle_min = 0.0f,
        .angle_max = 8192.0f,
        .gravity_compensation = 0.0f,
        .initial_angle = 1200.0f  // ← 更新为实际值
    },
},
{
    .motor_id = 6,
    // ... 其他配置 ...
    .limits.gm6020 = {
        .angle_min = 0.0f,
        .angle_max = 8192.0f,
        .gravity_compensation = 0.0f,
        .initial_angle = 2500.0f  // ← 更新为实际值
    },
},
```

#### 步骤5：重新编译和烧录
```powershell
cmake --preset Debug -DROBOT_TYPE=sentry_swerve -B build_sentry
cmake --build build_sentry
# 然后烧录
```

#### 步骤6：验证
1. 重新启动STM32
2. 观察启动日志：
   ```
   STM32> [INFO][SEN] Waiting for steer motors to align to initial position...
   STM32> [DEBUG][SEN] Steer motor 5: target=1200.0 current=1200.0 error=0.0
   STM32> [DEBUG][SEN] Steer motor 6: target=2500.0 current=2500.0 error=0.0
   STM32> [INFO][SEN] Steer alignment complete!
   ```
3. **观察轮子**：所有轮子应该自动对齐到同一个方向

---

### 方法2：使用代码读取当前角度（如果代码支持）

如果代码中有读取编码器的功能，可以：

1. 手动对齐轮子
2. 读取编码器值
3. 更新配置文件

---

## 📊 如何读取编码器值

### 方法A：通过日志输出

如果启用了调试日志，启动时会显示：

```
STM32> [DEBUG][SEN] Steer motor 5: target=1084.0 current=1234.0 error=150.0
```

这里的 `current=1234.0` 就是当前编码器值。

### 方法B：添加临时调试代码

在 `application/chassis/sentry_controller.c` 的 `Sentry_WaitForSteerAlignment()` 函数中，已经有日志输出。如果看不到，可以：

1. 确保 `LOG_ENABLE_SEN` 在 `logger_config.h` 中设置为 `1`
2. 重新编译并烧录

### 方法C：使用串口工具

如果STM32通过USB CDC输出日志，可以使用串口工具（如PuTTY、Tera Term）查看输出。

---

## ⚙️ 配置说明

### initial_angle 的含义

- **单位**：编码器ticks（0-8191，对应0-360度）
- **范围**：0.0f 到 8192.0f
- **作用**：定义轮子"前进"方向对应的编码器角度

### 编码器范围

- GM6020编码器：**8192 ticks = 360度**
- 1 tick ≈ 0.044度
- 校准精度：通常误差在 **±100 ticks** 内即可（约±4.4度）

---

## 🔄 自动对齐流程

代码启动时会自动执行对齐：

1. **初始化阶段**（`main.c` 第317行）：
   ```c
   #if defined(ROBOT_TYPE_sentry_swerve)
   Sentry_WaitForSteerAlignment();
   #endif
   ```

2. **对齐过程**（`sentry_controller.c`）：
   - 设置目标角度为 `initial_angle`
   - 发送PID控制命令，使电机转到目标角度
   - 检查误差是否小于100 ticks
   - 最多等待5秒

3. **完成标志**：
   ```
   STM32> [INFO][SEN] Steer alignment complete!
   ```

---

## ⚠️ 常见问题

### 问题1：对齐超时

**症状**：
```
STM32> [WARN][SEN] Steer alignment timeout, continuing anyway...
```

**原因**：
- `initial_angle` 值不正确
- 电机没有反馈（CAN总线问题）
- 机械卡住

**解决**：
1. 检查CAN总线连接
2. 检查电机是否正常反馈
3. 重新校准 `initial_angle`

### 问题2：轮子对齐但方向不对

**症状**：
- 对齐完成，但轮子指向错误的方向

**解决**：
- 调整 `initial_angle` 值（±2048 ticks = ±90度）
- 或者调整 `direction` 参数

### 问题3：两个转向电机角度差太大

**症状**：
- 两个转向电机无法同时对齐

**原因**：
- 机械安装问题
- 编码器零点不同

**解决**：
- 检查机械安装
- 分别校准每个电机的 `initial_angle`

---

## 📝 校准检查清单

- [ ] 底盘已架空
- [ ] 所有电机已连接并上电
- [ ] CAN总线正常通信
- [ ] 日志已启用（`LOG_ENABLE_SEN = 1`）
- [ ] 手动对齐轮子到"前进"方向
- [ ] 读取并记录编码器值
- [ ] 更新 `config/sentry_swerve.h` 中的 `initial_angle`
- [ ] 重新编译并烧录
- [ ] 验证启动时自动对齐成功
- [ ] 测试发送速度命令，验证轮子方向正确

---

## 🎯 快速校准命令

```powershell
# 1. 编译sentry配置
cmake --preset Debug -DROBOT_TYPE=sentry_swerve -B build_sentry
cmake --build build_sentry

# 2. 烧录（使用STM32CubeProgrammer）

# 3. 连接USB CDC，查看日志
python script/cmd_vel_forwarder.py --port COM3

# 4. 观察对齐日志，记录编码器值

# 5. 更新config/sentry_swerve.h中的initial_angle

# 6. 重新编译和烧录
```

---

## 💡 提示

1. **首次校准**：建议手动对齐轮子，然后读取编码器值
2. **微调**：如果对齐后方向略有偏差，可以微调 `initial_angle`（±50-100 ticks）
3. **验证**：校准后，发送前进命令（vx=0.2），所有轮子应该指向同一个方向
4. **备份**：校准好的值建议记录在注释中，方便以后参考

---

## 📞 需要帮助？

如果校准遇到问题：
1. 检查CAN总线通信是否正常
2. 检查电机反馈是否正常
3. 查看日志输出，确认对齐过程
4. 尝试手动调整 `initial_angle` 值
