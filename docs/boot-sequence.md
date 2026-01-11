# Boot Sequence and LED Indicators

This document describes the RoboMaster control firmware boot sequence and LED color indicators that help diagnose startup progress and issues.

## Overview

The C-Board LED provides visual feedback during the boot process, changing colors as different initialization stages complete. Each color represents a specific phase of the startup sequence.

## LED Color Reference

| Stage | LED Color | Meaning | Duration | Location |
|-------|-----------|---------|----------|----------|
| 1️⃣ Hardware Init | 🔴 **RED** | MCU peripherals initialized (GPIO, CAN, UART, etc.) | ~100ms | `main.c:229` |
| 2️⃣ CAN & Motors | 🟡 **YELLOW** | CAN bus started, motor drivers loaded | ~300ms | `main.c:258` |
| 3️⃣ Gimbal Align | 🔵 **CYAN** | Gimbal motors moving to initial position | 0-10s | `main.c:278` |
| 4️⃣ IMU Calibration | 🔵 **BLUE** | Gyro calibration (Infantry only, keep robot still) | ~3s | `main.c:293` |
| 5️⃣ Application Init | 🟣 **MAGENTA** | Command/Chassis/Shooter controllers initialized | ~200ms | `main.c:313` |
| 6️⃣ Swerve Align | 🟣 **MAGENTA** | Swerve steer motors aligning (Sentry only) | 0-5s | `main.c:326` |
| 7️⃣ Peripherals | ⚪ **WHITE** | Remote control, vision, IMU sensor initialized | ~500ms | `main.c:331` |
| 8️⃣ System Ready | 🟢 **GREEN** | Boot complete, entering main loop | Continuous | `main.c:357` |

## Boot Sequence by Robot Type

### Infantry Standard/Swerve

```
🔴 RED (Hardware)
  ↓
🟡 YELLOW (CAN & Motors)
  ↓
🔵 CYAN (Gimbal Alignment) ← May take 0-10 seconds
  ↓
🔵 BLUE (IMU Calibration) ← Keep robot still! ~3 seconds
  ↓
🟣 MAGENTA (Application Init)
  ↓
⚪ WHITE (Peripherals)
  ↓
🟢 GREEN (System Ready)
```

**Total time:** 2-15 seconds (depending on gimbal alignment)

### Sentry Swerve

```
🔴 RED (Hardware)
  ↓
🟡 YELLOW (CAN & Motors)
  ↓
🔵 CYAN (Gimbal Yaw Alignment) ← Usually fast (~100ms with auto-init)
  ↓
🟣 MAGENTA (Application Init + Swerve Steer Alignment) ← May take 0-5 seconds
  ↓
⚪ WHITE (Peripherals)
  ↓
🟢 GREEN (System Ready)
```

**Total time:** 1-7 seconds (depending on motor positions)

**Note:** Sentry skips IMU calibration (`enable_imu_calibration = 0` in config).

## Detailed Boot Sequence

### Phase 1: Hardware Initialization (RED)

**LED: 🔴 RED**

```c
HAL_Init();
SystemClock_Config();
MX_GPIO_Init();
MX_CAN1_Init();
MX_CAN2_Init();
MX_SPI1_Init();
MX_I2C3_Init();
MX_USART1_UART_Init();
MX_USART3_UART_Init();
MX_USB_DEVICE_Init();
// ... other STM32CubeMX generated init
```

**What's happening:**
- STM32 HAL library initialization
- System clock configuration (168 MHz)
- GPIO pins configured (including LED)
- CAN, SPI, I2C, UART peripherals enabled
- USB device stack started

**Duration:** ~100ms

### Phase 2: CAN & Motor Initialization (YELLOW)

**LED: 🟡 YELLOW**

```c
// Initialize message center (pub-sub system)
MsgCenter_Init(g_msg_queue, MSG_CENTER_QUEUE_LEN);

// Start CAN managers
CAN_Manager_Init(&can1_manager, CAN_CHANNEL_1, ...);
CAN_Manager_Init(&can2_manager, CAN_CHANNEL_2, ...);
CAN_Manager_Start(&can1_manager);
CAN_Manager_Start(&can2_manager);

// Load motor configurations from robot config
MotorDriver_ModuleInit();

// Initialize gimbal application (subscribes to CAN feedback)
GimbalApp_Init();

// Wait 200ms for CAN bus to stabilize
HAL_Delay(200);
```

**What's happening:**
- Publish-subscribe message center initialized
- CAN bus filters configured and activated
- Motor configurations loaded from `robot_config.h`
- Gimbal controller subscribes to motor feedback topics
- 200ms delay allows motors to send first feedback frames

**Duration:** ~300ms total (including 200ms stabilization delay)

### Phase 3: Gimbal Alignment (CYAN)

**LED: 🔵 CYAN**

```c
if (robot_cfg->gimbal_motor_count > 0) {
    Gimbal_WaitForAlignment();  // Max 10 second timeout
}
```

**What's happening:**
- Gimbal motors move to their initial positions
- **Infantry:** Yaw and pitch motors align to configured `initial_angle`
- **Sentry:** Yaw motor uses auto-init (stays at current position if `initial_angle = -1.0f`)
- Function checks error < 50 encoder ticks (~2.2 degrees)
- Exits immediately if motors already aligned

**Duration:**
- 0-100ms if `initial_angle = -1.0f` (auto-init, no movement)
- 1-10 seconds if motors need to move to specific position
- 10 seconds timeout if motors fail to align

**Debug output:** Check USB CDC for alignment progress messages.

### Phase 4: IMU Calibration (BLUE) - Infantry Only

**LED: 🔵 BLUE**

**Only runs if `enable_imu_calibration = 1` in robot config.**

```c
if (robot_cfg->enable_imu_calibration) {
    LED_SetRGB(0, 0, 1);  // BLUE
    gyro_calibrate();      // ~3 seconds
}
```

**What's happening:**
- Robot must remain completely still
- BMI088 gyroscope bias calibration
- Samples gyro data and calculates zero-point offset
- Gimbal motors actively hold position during calibration

**Duration:** ~3 seconds

**Important:**
- **Infantry:** This phase is active (LED turns blue)
- **Sentry:** This phase is **skipped** (no blue LED)

### Phase 5: Application Controllers (MAGENTA)

**LED: 🟣 MAGENTA**

```c
// Initialize application layer controllers
CmdController_Init();
ChassisApp_Init();
ShooterApp_Init();
```

**What's happening:**
- Command controller initializes (central coordinator)
- Chassis controller loads PID configs and subscribes to topics
- Shooter controller initializes (if configured)
- All controllers ready to receive commands

**Duration:** ~200ms

### Phase 6: Swerve Steer Alignment (MAGENTA) - Sentry Only

**LED: 🟣 MAGENTA** (continues from previous phase)

```c
#if defined(ROBOT_TYPE_sentry_swerve)
Sentry_WaitForSteerAlignment();  // Max 5 second timeout
#endif
```

**What's happening:**
- Two GM6020 swerve steer motors move to initial angles
- Function checks error < 100 encoder ticks (~4.4 degrees)
- Exits when both motors aligned or 5 second timeout

**Duration:**
- 1-5 seconds depending on initial motor positions
- Only for Sentry Swerve chassis

### Phase 7: Peripheral Initialization (WHITE)

**LED: ⚪ WHITE**

```c
// Remote control receiver
remote_control_init();

// Vision communication (if enabled)
VisionComm_Init();

// Wait for ESC boot (500ms)
HAL_Delay(WAIT_ESC_BOOT_MS);

// WT61C IMU sensor (UART)
WT61C_Init(&WT61C_UART_HANDLE);
HAL_UARTEx_ReceiveToIdle_DMA(...);
```

**What's happening:**
- DR16 remote control receiver started (USART3)
- Vision communication protocol initialized (USB CDC)
- 500ms delay for M3508 ESCs to complete boot
- WT61C backup IMU sensor initialized (USART1)

**Duration:** ~500ms (mostly ESC boot delay)

### Phase 8: System Ready (GREEN)

**LED: 🟢 GREEN**

```c
LOG_INFO(LOG_TAG_SYS, "=== System Ready ===");
LED_SetRGB(0, 1, 0);

// Enter main loop
while (1) {
    gyro_data_update(&sensor_data);
    CmdController_Task(current_tick);
    MsgCenter_Dispatch();
    Buzzer_Update();
    LED_SetRGB(0, 1, 0);  // Continuously set to green
    // ...
}
```

**What's happening:**
- All initialization complete
- Main control loop running at ~200Hz (5ms cycle)
- LED remains solid green during normal operation
- System ready to accept remote control commands

## Troubleshooting

### Boot Stuck at Specific Color

| Stuck at | Possible Issue | Solution |
|----------|----------------|----------|
| 🔴 **RED** | Hardware initialization failed | Check power supply, reflash firmware |
| 🟡 **YELLOW** | CAN bus not responding | Check motor CAN connections, verify motor power |
| 🔵 **CYAN** | Gimbal won't align | Check mechanical obstructions, verify motor IDs, check motor feedback on CAN |
| 🔵 **BLUE** | IMU calibration failed | Keep robot completely still, check BMI088 connections |
| 🟣 **MAGENTA** | Swerve steer stuck | Check steer motor mechanical range, verify motor feedback |
| ⚪ **WHITE** | Peripheral init failed | Check remote control connection (USART3), verify IMU sensor (USART1) |

### Boot Time Too Long

If boot consistently takes >10 seconds:

1. **Gimbal alignment slow:**
   - Set `initial_angle = -1.0f` in motor config to use auto-init (current position)
   - Reduces boot time from 5-10s to ~100ms

2. **Swerve steer alignment slow:**
   - Check mechanical friction on steer axes
   - Verify steer motor PID tuning (`sentry_swerve.h`)

3. **CAN bus delays:**
   - Verify motor power supply is stable
   - Check for loose CAN connectors
   - Ensure CAN termination resistors are present

### LED Never Turns Green

If the system seems to work but LED stays at an earlier color:

- This is a **bug** - the code should always reach the green LED
- Check if there's an infinite loop or hang in initialization
- Connect via USB and check debug output for error messages
- Verify the LED itself is functional (check RGB pins on GPIOH)

## Motor Initial Position Configuration

### Auto-Initialize vs. Fixed Position

Motors can be configured to either move to a fixed initial position or stay at their current position on boot.

**Example:** Gimbal yaw motor in `sentry_swerve.h`

```c
.limits.gm6020 = {
    .angle_min = 0.0f,
    .angle_max = 8192.0f,
    .gravity_compensation = 0.0f,
    .initial_angle = -1.0f  // Auto-init: use current position
}
```

| `initial_angle` value | Behavior | Boot time |
|-----------------------|----------|-----------|
| `>= 0.0f` (e.g., `0.0f`) | Motor moves to specified encoder position | 1-10 seconds |
| `< 0.0f` (e.g., `-1.0f`) | Motor stays at current position (auto-init) | ~100ms |

**Recommendation:** Use `initial_angle = -1.0f` for faster boot times unless specific initial position is required.

### How Auto-Init Works

1. During `MotorDriver_Init()`, if `initial_angle < 0`:
   ```c
   ctx->angle_initialized = false;  // Mark as not initialized
   ```

2. On first CAN feedback frame:
   ```c
   if (!ctx->angle_initialized) {
       ctx->angle_target = (float)angle_raw;  // Set target = current
       ctx->angle_initialized = true;
   }
   ```

3. During `Gimbal_WaitForAlignment()`:
   ```c
   float error = fabsf(angle_target - angle_raw);  // ~0 error
   if (error < 50.0f) {
       return;  // Immediately pass
   }
   ```

Result: Motor doesn't move, alignment check passes in first iteration (~100ms).

## Pre-Boot Motor Behavior

**Observation:** GM6020 motors may shake/vibrate before C-Board powers on.

**Cause:** This is **normal** ESC behavior:
- When no CAN commands are received, GM6020 ESC enters protection mode
- ESC may attempt to hold position with last known parameters
- Capacitor discharge can briefly power motors during power-down

**Solution:** This is a hardware characteristic and cannot be eliminated in firmware. The shaking stops once the C-Board boots and sends valid CAN commands (~1 second after power-on).

## Implementation Details

### LED Control Function

```c
static void LED_SetRGB(uint8_t r, uint8_t g, uint8_t b)
{
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12, r ? GPIO_PIN_SET : GPIO_PIN_RESET); // R
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_11, g ? GPIO_PIN_SET : GPIO_PIN_RESET); // G
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_10, b ? GPIO_PIN_SET : GPIO_PIN_RESET); // B
}
```

The LED is a common-cathode RGB LED with three pins:
- **Red:** GPIOH Pin 12
- **Green:** GPIOH Pin 11
- **Blue:** GPIOH Pin 10

Colors are binary (on/off), creating 8 possible states:
- `(0,0,0)` = Off
- `(1,0,0)` = Red
- `(1,1,0)` = Yellow
- `(0,1,0)` = Green
- `(0,1,1)` = Cyan
- `(0,0,1)` = Blue
- `(1,0,1)` = Magenta
- `(1,1,1)` = White

### Boot Sequence Code Location

All boot sequence code is in `Src/main.c` between lines 226-357 (within `USER CODE BEGIN 2` section).

This code is preserved when regenerating from STM32CubeMX.

## Related Documentation

- **Architecture:** See `docs/architecture.md` for system overview
- **Motor Configuration:** See robot config files in `config/` directory
- **Message Center:** See `docs/pub-sub.md` for publish-subscribe system
- **Logger System:** See `docs/logger-guide.md` for debug output

## Testing Recommendations

### Verify Boot Sequence

1. Power on robot and observe LED colors
2. Expected sequence should match robot type (see above)
3. Boot should complete in 1-15 seconds depending on motor positions
4. Final state should be solid green LED

### Measure Boot Time

Add timing code to measure each phase:

```c
uint32_t t_start = HAL_GetTick();
// ... initialization phase ...
uint32_t t_elapsed = HAL_GetTick() - t_start;
LOG_INFO(LOG_TAG_SYS, "Phase X took %lu ms", t_elapsed);
```

### Debug Boot Issues

1. **Connect USB cable** to view log output
2. **Monitor LED colors** to identify stuck phase
3. **Check CAN feedback** using `LOG_ENABLE_CAN = 1` in `logger_config.h`
4. **Verify motor connections** with `LOG_ENABLE_MOT = 1`

---

**Last updated:** 2026-01-11
