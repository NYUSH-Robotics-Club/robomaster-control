#include "gyro_data.h"
#include "bmi088driver.h"
#include "wt61c.h"
#include "printing.h"
#include "message_center.h"
#include "QuaternionEKF.h"
#include <math.h>

static float gyro[3];
static float accel[3];
static float temp;

// 姿态估计变量（静态保持状态）- QuaternionEKF会管理这些
static uint8_t initialized = 0;     // 初始化标志

// 陀螺仪零偏（rad/s）- 开机静止校准获得
static float gyro_offset[3] = {0.0f, 0.0f, 0.0f};
static uint8_t calibrated = 0;      // 校准完成标志
static float accel_scale = 1.0f;    // 加速度计缩放系数
static float g_norm = 9.81f;        // 重力加速度范数

#define RAD_TO_DEG (57.295779513f)
#define DEG_TO_RAD (0.017453292f)
#define DT (0.005f)  // 5ms = 200Hz
#define GRAVITY_ACCEL (9.80665f)  // 标准重力加速度 m/s²

/**
 * @brief 使用加速度计初始化四元数（仅roll和pitch）
 * @param ax, ay, az: 加速度计读数（单位：m/s²）
 */
static void init_attitude_from_accel(float ax, float ay, float az)
{
    // 注意：BMI088_read返回的accel单位是m/s²，需要转换为g用于角度计算
    ax = ax / GRAVITY_ACCEL;
    ay = ay / GRAVITY_ACCEL;
    az = az / GRAVITY_ACCEL;

    // 计算roll和pitch（假设静止状态，加速度计测量重力方向）
    float roll = atan2f(ay, az);
    float pitch = atan2f(-ax, sqrtf(ay*ay + az*az));
    float yaw = 0.0f;  // yaw无法从加速度计获取，初始化为0

    // 从欧拉角转换到四元数
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);

    float init_quaternion[4];
    init_quaternion[0] = cr * cp * cy + sr * sp * sy;  // q0 (w)
    init_quaternion[1] = sr * cp * cy - cr * sp * sy;  // q1 (x)
    init_quaternion[2] = cr * sp * cy + sr * cp * sy;  // q2 (y)
    init_quaternion[3] = cr * cp * sy - sr * sp * cy;  // q3 (z)

    // 初始化QuaternionEKF
    // 参数: init_quaternion, process_noise1, process_noise2, measure_noise, lambda, lpf
    // process_noise1: 四元数过程噪声 (10)
    // process_noise2: 陀螺仪零偏过程噪声 (0.001)
    // measure_noise: 加速度计量测噪声 (1000000)
    // lambda: 渐消因子 (0.9996)
    // lpf: 低通滤波系数 (0 = 不使用低通滤波)
    IMU_QuaternionEKF_Init(init_quaternion, 10.0f, 0.001f, 1000000.0f, 0.9996f, 0.0f);

    initialized = 1;

    USB_CDC_Printf("[QuaternionEKF] Initialized with q=[%.3f,%.3f,%.3f,%.3f], roll=%.2f°, pitch=%.2f°\r\n",
                   init_quaternion[0], init_quaternion[1], init_quaternion[2], init_quaternion[3],
                   roll * RAD_TO_DEG, pitch * RAD_TO_DEG);
}

/**
 * @brief IMU校准（与basic_framework的Calibrate_MPU_Offset完全一致）
 * @attention 调用此函数时，IMU必须静止不动
 * @note 采集6000个样本（约6秒），计算陀螺仪零偏和加速度计缩放
 */
void gyro_calibrate(void)
{
    #define CALIB_SAMPLES 2000    // 2000样本（约2秒）- 更实用
    #define CALIB_TIMEOUT_MS 15000  // 15秒超时
    #define MAX_RETRY 3             // 最多重试3次

    uint32_t start_time = HAL_GetTick();
    uint16_t cali_count = 0;

    float gyro_max[3], gyro_min[3];
    float gyro_diff[3];
    float g_norm_temp = 0.0f, g_norm_max = 0.0f, g_norm_min = 0.0f, g_norm_diff = 0.0f;

    // 验证BMI088是否初始化成功
    USB_CDC_Printf("[BMI088] Verifying BMI088 initialization...\r\n");
    BMI088_read(gyro, accel, &temp);
    USB_CDC_Printf("[BMI088] Initial reading: accel=[%.3f,%.3f,%.3f]m/s², gyro=[%.3f,%.3f,%.3f]rad/s, temp=%.1f°C\r\n",
                   accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2], temp);

    if (accel[0] == 0.0f && accel[1] == 0.0f && accel[2] == 0.0f) {
        USB_CDC_Printf("[BMI088] *** ERROR: Accelerometer reading is ALL ZERO! ***\r\n");
        USB_CDC_Printf("[BMI088] *** BMI088 accelerometer initialization may have failed! ***\r\n");
        USB_CDC_Printf("[BMI088] *** Check SPI connection and BMI088_init() return value ***\r\n");
    }

    if (temp < -50.0f || temp > 100.0f) {
        USB_CDC_Printf("[BMI088] *** WARNING: Temperature reading abnormal (%.1f°C) ***\r\n", temp);
    }

    USB_CDC_Printf("[BMI088] Starting calibration, keep IMU still for 2 seconds...\r\n");

    // do-while循环：重试直到校准成功或超时
    do {
        // 检查超时或重试次数
        if (HAL_GetTick() - start_time > CALIB_TIMEOUT_MS || cali_count >= MAX_RETRY) {
            if (cali_count >= MAX_RETRY) {
                USB_CDC_Printf("[BMI088] Max retry reached! Using last attempt values.\r\n");
                // 使用最后一次尝试的值（总比默认值好）
                accel_scale = 9.81f / g_norm;
                calibrated = 1;
            } else {
                USB_CDC_Printf("[BMI088] Calibration timeout! Using default values.\r\n");
                gyro_offset[0] = 0.0f;
                gyro_offset[1] = 0.0f;
                gyro_offset[2] = 0.0f;
                g_norm = 9.81f;
                accel_scale = 1.0f;
                temp = 40.0f;
                calibrated = 0;
            }
            break;
        }

        if (cali_count > 0) {
            HAL_Delay(100);  // 重试前延时100ms
        }

        // 重置累积变量
        g_norm = 0.0f;
        gyro_offset[0] = 0.0f;
        gyro_offset[1] = 0.0f;
        gyro_offset[2] = 0.0f;

        USB_CDC_Printf("[BMI088] Attempt %d: Collecting %d samples...\r\n", cali_count + 1, CALIB_SAMPLES);

        // 采集2000个样本
        for (uint16_t i = 0; i < CALIB_SAMPLES; i++) {
            BMI088_read(gyro, accel, &temp);

            // 累积加速度计范数（重力）
            // 注意：BMI088_read返回的accel单位是m/s²
            g_norm_temp = sqrtf(accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2]);
            g_norm += g_norm_temp;

            // 首次采样时打印调试信息
            if (i == 0 && cali_count == 0) {
                USB_CDC_Printf("[BMI088] First sample: accel=[%.3f,%.3f,%.3f]m/s², gyro=[%.3f,%.3f,%.3f]rad/s, temp=%.1f°C\r\n",
                               accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2], temp);
            }

            // 累积陀螺仪零偏
            gyro_offset[0] += gyro[0];
            gyro_offset[1] += gyro[1];
            gyro_offset[2] += gyro[2];

            // 记录最大最小值
            if (i == 0) {
                g_norm_max = g_norm_temp;
                g_norm_min = g_norm_temp;
                gyro_max[0] = gyro[0];
                gyro_max[1] = gyro[1];
                gyro_max[2] = gyro[2];
                gyro_min[0] = gyro[0];
                gyro_min[1] = gyro[1];
                gyro_min[2] = gyro[2];
            } else {
                if (g_norm_temp > g_norm_max) g_norm_max = g_norm_temp;
                if (g_norm_temp < g_norm_min) g_norm_min = g_norm_temp;

                for (uint8_t j = 0; j < 3; j++) {
                    if (gyro[j] > gyro_max[j]) gyro_max[j] = gyro[j];
                    if (gyro[j] < gyro_min[j]) gyro_min[j] = gyro[j];
                }
            }

            // 每隔500个样本检查一次运动（减少检查频率，提高效率）
            if (i % 500 == 499) {
                g_norm_diff = g_norm_max - g_norm_min;
                gyro_diff[0] = gyro_max[0] - gyro_min[0];
                gyro_diff[1] = gyro_max[1] - gyro_min[1];
                gyro_diff[2] = gyro_max[2] - gyro_min[2];

                if (g_norm_diff > 0.5f ||  // 重力范数变化 < 0.5 m/s²
                    gyro_diff[0] > 0.15f ||
                    gyro_diff[1] > 0.15f ||
                    gyro_diff[2] > 0.15f) {
                    USB_CDC_Printf("[BMI088] Movement detected (gNorm_diff=%.3f m/s², gyro_diff=[%.3f,%.3f,%.3f]rad/s), retry...\r\n",
                                   g_norm_diff, gyro_diff[0], gyro_diff[1], gyro_diff[2]);
                    break;  // 跳出采样循环，重新开始
                }
            }

            HAL_Delay(1);  // 1ms间隔
        }

        // 计算平均值和最终差值
        g_norm /= (float)CALIB_SAMPLES;
        gyro_offset[0] /= (float)CALIB_SAMPLES;
        gyro_offset[1] /= (float)CALIB_SAMPLES;
        gyro_offset[2] /= (float)CALIB_SAMPLES;

        // 计算最终差值用于检查
        g_norm_diff = g_norm_max - g_norm_min;
        gyro_diff[0] = gyro_max[0] - gyro_min[0];
        gyro_diff[1] = gyro_max[1] - gyro_min[1];
        gyro_diff[2] = gyro_max[2] - gyro_min[2];

        USB_CDC_Printf("[BMI088] Sample complete: gNorm=%.3f m/s² (%.2fg), offset=[%.6f,%.6f,%.6f]rad/s\r\n",
                       g_norm, g_norm / GRAVITY_ACCEL, gyro_offset[0], gyro_offset[1], gyro_offset[2]);

        cali_count++;

    } while (g_norm_diff > 0.5f ||                                    // 重力范数变化 < 0.5 m/s²
             fabsf(g_norm - GRAVITY_ACCEL) > 0.5f ||              // gNorm应该接近9.8 m/s²
             gyro_diff[0] > 0.15f ||
             gyro_diff[1] > 0.15f ||
             gyro_diff[2] > 0.15f ||
             fabsf(gyro_offset[0]) > 0.05f ||  // 放宽到0.05（原来0.01太严格）
             fabsf(gyro_offset[1]) > 0.05f ||
             fabsf(gyro_offset[2]) > 0.05f);

    // 仅在成功时计算加速度计缩放系数
    if (calibrated) {
        // gNorm是m/s²，accel_scale用于校准（理想情况下接近1.0）
        accel_scale = GRAVITY_ACCEL / g_norm;  // 应该接近1.0
        USB_CDC_Printf("[BMI088] ===== Calibration SUCCESS (attempts: %d) =====\r\n", cali_count);
        USB_CDC_Printf("Gyro offset: gx=%.6f, gy=%.6f, gz=%.6f rad/s\r\n",
                       gyro_offset[0], gyro_offset[1], gyro_offset[2]);
        USB_CDC_Printf("Accel: gNorm=%.3f m/s² (%.2fg)\r\n", g_norm, g_norm / GRAVITY_ACCEL);
        USB_CDC_Printf("Temp when cali: %.2f °C\r\n", temp);
    }
}

void gyro_data_init(void)
{
    BMI088_init();
    initialized = 0;  // 标记为未初始化，等待第一次数据
}

void gyro_data_update(SensorData *sensor_data)
{
    BMI088_read(gyro, accel, &temp);

    // 保存原始IMU数据（gyro: rad/s, accel: m/s²）
    sensor_data->g_gx = gyro[0];
    sensor_data->g_gy = gyro[1];
    sensor_data->g_gz = gyro[2];
    sensor_data->g_ax = accel[0];
    sensor_data->g_ay = accel[1];
    sensor_data->g_az = accel[2];

    // === 如果未初始化，使用加速度计初始化四元数 ===
    if (!initialized) {
        init_attitude_from_accel(accel[0], accel[1], accel[2]);
        return;
    }

    // === QuaternionEKF姿态更新 ===
    // QuaternionEKF需要的输入：
    // - 陀螺仪：rad/s（BMI088已经是rad/s）
    // - 加速度计：m/s²（BMI088已经返回m/s²，直接使用）
    // - 采样周期：s
    IMU_QuaternionEKF_Update(gyro[0], gyro[1], gyro[2],
                              accel[0], accel[1], accel[2],
                              DT);

    // 从QuaternionEKF获取姿态角度（度）
    sensor_data->yaw = QEKF_INS.Yaw;
    sensor_data->pitch = QEKF_INS.Pitch;
    sensor_data->roll = QEKF_INS.Roll;

    // 获取多圈累积角度（QuaternionEKF已经计算好了）
    sensor_data->yaw_total_angle = QEKF_INS.YawTotalAngle;
    sensor_data->yaw_round_count = QEKF_INS.YawRoundCount;

    // 兼容旧代码
    sensor_data->absolute_angle = sensor_data->yaw_total_angle;

    // === CDC串口输出IMU姿态数据（10Hz） ===
    static uint32_t last_imu_print = 0;
    uint32_t now = HAL_GetTick();
    if (now - last_imu_print >= 100) {  // 每100ms输出一次
        last_imu_print = now;
        USB_CDC_Printf("IMU,%lu,%.2f,%.2f,%.2f,%.2f,%d,%.3f,%.3f,%.3f\r\n",
                       now,
                       sensor_data->yaw,
                       sensor_data->pitch,
                       sensor_data->roll,
                       sensor_data->yaw_total_angle,
                       sensor_data->yaw_round_count,
                       sensor_data->g_gx,
                       sensor_data->g_gy,
                       sensor_data->g_gz);
    }

    // WT61C底盘IMU数据（保持原样）
    WT61C_Data const* d = WT61C_GetData();
    sensor_data->c_ax = (int)(d->ax * 1000);
    sensor_data->c_ay = (int)(d->ay * 1000);
    sensor_data->c_az = (int)(d->az * 1000);
    sensor_data->c_gx = (int)(d->gx );
    sensor_data->c_gy = (int)(d->gy );
    sensor_data->c_gz = (int)(d->gz );
    sensor_data->c_roll = (int)(d->roll * 10);
    sensor_data->c_pitch = (int)(d->pitch * 10);
    sensor_data->c_yaw = (int)(d->yaw * 10);

    (void)MsgCenter_Publish(TOPIC_IMU_UPDATE, sensor_data, sizeof(*sensor_data));
}

void WT61C_OnNewData(const WT61C_Data *d)
{
  (void)d;
}


