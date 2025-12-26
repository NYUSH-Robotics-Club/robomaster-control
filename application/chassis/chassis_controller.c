#include "chassis_controller.h"
#include "can.h"
#include "can_manager.h"
#include <string.h>
#include <math.h>
#include "message_center.h"
#include "remote_control.h"
#include "gyro_data.h"
#include "can_comm.h"
#include "printing.h"
#include "cmd_controller.h"
#include "gm6020_motor.h"
#include "pid.h"
#include "message_center.h"
#include "can.h"
#include "can_manager.h"
#include <math.h>
#include <string.h>
#include "printing.h"
#include "stm32f4xx_hal.h"

#define MECANUM_WHEEL_BASE_MODE
//#define STEERING_WHEEL_BASE_MODE    




#if defined(MECANUM_WHEEL_BASE_MODE)

    extern CAN_HandleTypeDef hcan1;

    #define SPEED_PID_KP (5.0f)
    #define SPEED_PID_KI (0.5f)
    #define SPEED_PID_KD (0.1f)
    #define SPEED_PID_OUTPUT_MAX (15000)
    #define SPEED_PID_INTEGRAL_MAX (3000)
    #define MOTOR_FEEDBACK_TIMEOUT_MS (100U)
    #define MOTOR_STDID_1_4 (0x200U)

    typedef struct { float x; float y; } Pair;

    // Static variables for app wrapper
    static ChassisCmd s_last_cmd;
    static SensorData s_last_sensor;
    static ChassisController s_ctrl;

    static Pair to_real_speed(Pair speed, float angle, float w) {
        const float k = 0.01f;
        angle += k * w;
        Pair result;
        result.x = speed.x * cosf(angle) - speed.y * sinf(angle);
        result.y = speed.x * sinf(angle) + speed.y * cosf(angle);
        return result;
    }

    static const int8_t MOTOR_DIR[CHASSIS_MOTOR_COUNT] = { -1, +1, +1, -1 };

    static float RampTowards(float current, float target, float step)
    {
        if (current < target) { current += step; if (current > target) current = target; }
        else if (current > target) { current -= step; if (current < target) current = target; }
        return current;
    }

    static void ResetPidIntegrals(ChassisController *controller)
    {
        for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++) { controller->speed_pids[i].integral = 0.0f; }
    }

    static int16_t ComputeSingleMotorCurrent(PID_Controller *pid, float target, Motor_Feedback *feedback, uint32_t current_tick)
    {
        if (current_tick - feedback->last_update_time > MOTOR_FEEDBACK_TIMEOUT_MS) { return 0; }
        float current_speed = feedback->speed;
        return (int16_t)PID_Calculate(pid, target, current_speed);
    }

    void ChassisController_Init(ChassisController *controller)
    {
        if (controller == NULL) return;
        memset(controller, 0, sizeof(ChassisController));
        for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
            PID_Init(&controller->speed_pids[i], SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD, 
                    SPEED_PID_OUTPUT_MAX, SPEED_PID_INTEGRAL_MAX);
        }
        for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
            controller->target_speeds[i] = 0.0f;
            controller->ramped_targets[i] = 0.0f;
        }
    }

    void ChassisController_Update(ChassisController *controller, SensorData* sensor_data)
    {
        if (controller == NULL) return;
        
        float vx_norm = s_last_cmd.vx;
        float vy_norm = s_last_cmd.vy;
        float wz_norm = s_last_cmd.wz;
        float scale = (float)CHASSIS_DEMO_TARGET_SPEED / 2.0f;
        float omega = wz_norm * scale;
        Pair _speed = (Pair){vx_norm * scale, vy_norm * scale};
        Pair speed = to_real_speed(_speed, sensor_data->c_yaw, omega);
        float vx = speed.x, vy = speed.y;

        controller->target_speeds[0] = MOTOR_DIR[0] * (vx - vy + omega);
        controller->target_speeds[1] = MOTOR_DIR[1] * (vx + vy - omega);
        controller->target_speeds[2] = MOTOR_DIR[2] * (vx - vy - omega);
        controller->target_speeds[3] = MOTOR_DIR[3] * (vx + vy + omega);
        
        controller->running = s_last_cmd.enabled;
        
        for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
            controller->ramped_targets[i] = RampTowards(controller->ramped_targets[i], controller->target_speeds[i], CHASSIS_RAMP_STEP);
        }
    }

    void ChassisController_ComputeCurrents(ChassisController *controller, uint32_t current_tick)
    {
        if (controller == NULL) return;
        for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
            int16_t motor_current = ComputeSingleMotorCurrent(
                &controller->speed_pids[i],
                controller->ramped_targets[i],
                &controller->motor_feedbacks[i],
                current_tick
            );
            controller->output_currents[i] = motor_current;
        }
        CAN_Manager_SendMotorCurrents4(&hcan1, MOTOR_STDID_1_4,
            controller->output_currents[0], controller->output_currents[1], controller->output_currents[2], controller->output_currents[3]);
    }

    void ChassisController_SetTargetSpeeds(ChassisController *controller, const float speeds[CHASSIS_MOTOR_COUNT])
    {
        if (controller == NULL || speeds == NULL) return;
        for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++) { controller->target_speeds[i] = speeds[i]; }
    }

    void ChassisController_Stop(ChassisController *controller)
    {
        if (controller == NULL) return;
        controller->running = false;
        ResetPidIntegrals(controller);
        for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++) { controller->target_speeds[i] = 0.0f; }
    }

    const int16_t* ChassisController_GetOutputCurrents(const ChassisController *controller)
    {
        if (controller == NULL) return NULL;
        return controller->output_currents;
    }

    bool ChassisController_IsRunning(const ChassisController *controller)
    {
        if (controller == NULL) return false;
        for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++) { if (controller->ramped_targets[i] != 0) { return true; } }
        return false;
    }

    void ChassisController_UpdateMotorFeedback(ChassisController *controller, uint8_t motor_id, uint16_t angle, int16_t speed, int16_t current, uint8_t temp, uint32_t current_tick)
    {
        if (controller == NULL || motor_id >= CHASSIS_MOTOR_COUNT) return;
        controller->motor_feedbacks[motor_id].angle = angle;
        controller->motor_feedbacks[motor_id].speed = speed;
        controller->motor_feedbacks[motor_id].current = current;
        controller->motor_feedbacks[motor_id].temp = temp;
        controller->motor_feedbacks[motor_id].last_update_time = current_tick;
    }

    // Subscription callbacks
    static void on_chassis_cmd(const MsgEvent *ev, void *user) {
        (void)user;
        if (ev->size == sizeof(ChassisCmd)) {
            memcpy(&s_last_cmd, ev->data, sizeof(ChassisCmd));
            // Update controller and compute currents when command arrives
            ChassisController_Update(&s_ctrl, &s_last_sensor);
            ChassisController_ComputeCurrents(&s_ctrl, HAL_GetTick());
        }
    }

    static void on_imu_update(const MsgEvent *ev, void *user) {
        (void)user;
        if (ev->size == sizeof(SensorData)) {
            memcpy(&s_last_sensor, ev->data, sizeof(SensorData));
        }
    }

    static void on_motor_feedback(const MsgEvent *ev, void *user) {
        (void)user;
        if (ev->size == sizeof(MotorFeedbackEvent)) {
            const MotorFeedbackEvent *m = (const MotorFeedbackEvent *)ev->data;
            // Only process chassis motor feedback (id < 4)
            if (m->id < 4) {
                ChassisController_UpdateMotorFeedback(&s_ctrl, m->id, m->angle, m->speed, m->current, m->temp, m->tick_ms);
            }
        }
    }

    void ChassisApp_Init(void) {
        memset(&s_last_cmd, 0, sizeof(s_last_cmd));
        memset(&s_last_sensor, 0, sizeof(s_last_sensor));
        ChassisController_Init(&s_ctrl);
        (void)MsgCenter_Subscribe(TOPIC_CHASSIS_CMD, on_chassis_cmd, NULL);
        (void)MsgCenter_Subscribe(TOPIC_IMU_UPDATE, on_imu_update, NULL);
        (void)MsgCenter_Subscribe(TOPIC_MOTOR_FEEDBACK, on_motor_feedback, NULL);
    }

    ChassisController* ChassisApp_GetController(void) {
        return &s_ctrl;
    }







#elif defined(STEERING_WHEEL_BASE_MODE)
    #define Radius 0.21f               //轮子半径 未测量 所有数据均待调试
    #define CHASSIS_MOTOR_COUNT 6
    #define MOTOR_STDID_1_4 (0x200U)//待定
    #define Chasis_GM6020_ID_1 (0x20AU)//待定
    #define Chasis_GM6020_ID_2 (0x20BU)//待定
    #define GM6020_KP (10.0f)
    #define GM6020_KI (0.5f)
    #define GM6020_KD (0.1f)
    #define GM6020_ANGLE_KP (8.0f)
    #define GM6020_ANGLE_KI (0.3f)
    #define GM6020_ANGLE_KD (0.05f)
    #define GM6020_ANGLE_OUTPUT_MAX (15000)
    #define GM6020_ANGLE_INTEGRAL_MAX (3000)
    #define M3508_KP (5.0f)
    #define M3508_KI (0.3f)
    #define M3508_KD (0.05f)
    #define M3508_OUTPUT_MAX (20000)
    #define M3508_INTEGRAL_MAX (4000)
    #define GM6020_OUTPUT_MAX (15000)
    #define GM6020_INTEGRAL_MAX (3000)       //减速比
    #define GM6020_FL_ANGLE 23434		//装上去的时候Y轴正向对应的编码值(偏置角度)(需要调试)
    #define GM6020_FR_ANGLE 26985
    #define GM6020_BL_ANGLE 344
    #define GM6020_BR_ANGLE 13310
    typedef struct {
    // Motor target speeds
    float target_speeds[CHASSIS_MOTOR_COUNT];
    // Smoothed target speeds
    float ramped_targets[CHASSIS_MOTOR_COUNT];
    // Running state
    bool running;
    // PID controllers
    PID_Controller speed_pids[CHASSIS_MOTOR_COUNT];
    // Motor feedbacks
    Motor_Feedback motor_feedbacks[CHASSIS_MOTOR_COUNT];
    // Output currents
    int16_t output_currents[CHASSIS_MOTOR_COUNT];
} ChassisController;

   
    fp32 PID_CurrentLT1, PID_CurrentLT2, PID_CurrentRT1, PID_CurrentRT2;
    fp32 PID_GM6020_Speed_Current_FL, PID_GM6020_Speed_Current_FR, PID_GM6020_Speed_Current_BL, PID_GM6020_Speed_Current_BR;
    fp32 PID_GM6020_Current_FL, PID_GM6020_Current_FR, PID_GM6020_Current_BL, PID_GM6020_Current_BR;
    fp32 M3508_SPEED[4], GM6020_ANGLE[2];
    fp32 GM6020_speed[4];

    fp32 relative_angle_set;

    fp32 vx_set, vy_set, wz_set;
    float Angle_Limit (float angle ,float max)
    {
            if(angle > max)
                angle -= max;
            if(angle < 0)
                angle += max; 
            return angle;
    }
   
    void ChassisController_UpdateMotorFeedback(ChassisController *controller, uint8_t motor_id, uint16_t angle, int16_t speed, int16_t current, uint8_t temp, uint32_t current_tick)
    {
        if (controller == NULL || motor_id >= CHASSIS_MOTOR_COUNT) return;
        controller->motor_feedbacks[motor_id].angle = angle;
        controller->motor_feedbacks[motor_id].speed = speed;
        controller->motor_feedbacks[motor_id].current = current;
        controller->motor_feedbacks[motor_id].temp = temp;
        controller->motor_feedbacks[motor_id].last_update_time = current_tick;
    }
    


    
    static void on_imu_update(const MsgEvent *ev, void *user) {
        (void)user;
        if (ev->size == sizeof(SensorData)) {
            memcpy(&s_last_sensor, ev->data, sizeof(SensorData));
        }
    }

    static void on_motor_feedback(const MsgEvent *ev, void *user) {
        (void)user;
        if (ev->size == sizeof(MotorFeedbackEvent)) {
            const MotorFeedbackEvent *m = (const MotorFeedbackEvent *)ev->data;
            // Only process chassis motor feedback (id < 4)
            if (m->id < CHASSIS_MOTOR_COUNT) {
                ChassisController_UpdateMotorFeedback(&s_ctrl, m->id, m->angle, m->speed, m->current, m->temp, m->tick_ms);
            }
        }
    }

    //将电机转子转向内侧时 修正方向
    int8_t dirt[4] = { 1, -1, 1, -1};
    void chassis_vector_to_M3508_wheel_speed(fp32 vx_set, fp32 vy_set, fp32 wz_set, fp32 wheel_speed[4])
    {
        fp32 wheel_rpm_ratio;
        
        wheel_rpm_ratio = 60.0f / (WHEEL_PERIMETER * 3.14159f) * M3508_RATIO * 1000;

        wheel_speed[0] = dirt[0] * sqrt(	pow(vy_set + wz_set * Radius * 0.707107f,2)
                        +	pow(vx_set - wz_set * Radius * 0.707107f,2)
                        ) * wheel_rpm_ratio ;
        wheel_speed[1] = dirt[1] * sqrt(	pow(vy_set - wz_set * Radius * 0.707107f,2)
                        +	pow(vx_set - wz_set * Radius * 0.707107f,2)
                        ) * wheel_rpm_ratio ;
        wheel_speed[2] = dirt[2] * sqrt(	pow(vy_set - wz_set * Radius * 0.707107f,2)
                        +	pow(vx_set + wz_set * Radius * 0.707107f,2)
                        ) * wheel_rpm_ratio ;
        wheel_speed[3] = dirt[3] * sqrt(	pow(vy_set + wz_set * Radius * 0.707107f,2)
                        +	pow(vx_set + wz_set * Radius * 0.707107f,2) 
                        ) * wheel_rpm_ratio ;
            
    }  

    fp64 atan_angle[4];
    void chassis_vector_to_GM6020_wheel_angle(fp32 vx_set, fp32 vy_set, fp32 wz_set, fp32 wheel_angle[4])
    {

        
            //6020目标角度计算
        if(!(vx_set == 0 && vy_set == 0 && wz_set == 0))//防止除数为零
        {
                //由于atan2算出来的结果是弧度，需转换成角度 计算公式为 弧度 * 180.f / PI 最终得到角度值 (0.707107f == 根号2)
        atan_angle[0] = atan2((vx_set - wz_set * Radius * 0.707107f),(vy_set + wz_set * Radius * 0.707107f)) * 180.0f / PI;		
        atan_angle[1] = atan2((vx_set - wz_set * Radius * 0.707107f),(vy_set - wz_set * Radius * 0.707107f)) * 180.0f / PI;
        atan_angle[2] = atan2((vx_set + wz_set * Radius * 0.707107f),(vy_set + wz_set * Radius * 0.707107f)) * 180.0f / PI;
        atan_angle[3] = atan2((vx_set + wz_set * Radius * 0.707107f),(vy_set - wz_set * Radius * 0.707107f)) * 180.0f / PI;	
        }  
            
            // 将一圈360°转换成编码值的一圈0-32767 -> 角度 * 32767 / 360 最终转换为需要转动的角度对应的编码值，再加上偏置角度,最终得到目标编码值
            wheel_angle[0] = Angle_Limit(GM6020_FL_ANGLE + (fp32)(atan_angle[0] * 91.02f), 32767.f);
            wheel_angle[1] = Angle_Limit(GM6020_FR_ANGLE + (fp32)(atan_angle[1] * 91.02f), 32767.f);
            wheel_angle[2] = Angle_Limit(GM6020_BL_ANGLE + (fp32)(atan_angle[2] * 91.02f), 32767.f);
            wheel_angle[3] = Angle_Limit(GM6020_BR_ANGLE + (fp32)(atan_angle[3] * 91.02f), 32767.f);
            
            //优弧 劣弧 驱动电机转向判断
        if( ABS( motor_feedbacks[0].angle - wheel_angle[0] ) > 8192 )
        {	
                dirt[0] = -1;
                wheel_angle[0] = Angle_Limit( wheel_angle[0] - 16384, 32767 );
        }
        else
            dirt[0] = 1;
            
    if( ABS( motor_feedbacks[1].angle - wheel_angle[1] ) > 8192 )
        {	
                dirt[1] = 1;
                wheel_angle[1] = Angle_Limit( wheel_angle[1] - 16384, 32767 );
        }
        else
            dirt[1] = -1;

    if( ABS( motor_feedbacks[2].angle - wheel_angle[2] ) > 8192 )
        {	
                dirt[2] = -1;
                wheel_angle[2] = Angle_Limit( wheel_angle[2] - 16384, 32767 );
        }
        else
            dirt[2] = 1;

    if( ABS( motor_feedbacks[3].angle - wheel_angle[3] ) > 8192 )
        {	
                dirt[3] = 1;
                wheel_angle[3] = Angle_Limit( wheel_angle[3] - 16384, 32767 );
        }
        else
            dirt[3] = -1;//0 1 2 3 是驱动电机
        
    }
    
    void ChassisController_Init(ChassisController *controller)
    {
        if (controller == NULL) return;
        memset(controller, 0, sizeof(ChassisController));
        for (int i = 0; i < CHASSIS_MOTOR_COUNT-2; i++) {
            PID_Init(&controller->speed_pids[i], M3508_KP, M3508_KI, M3508_KD, 
                    M3508_OUTPUT_MAX, M3508_INTEGRAL_MAX);
        }
        for (int i = CHASSIS_MOTOR_COUNT-2; i < CHASSIS_MOTOR_COUNT; i++) {
            PID_Init(&controller->speed_pids[i], GM6020_KP, GM6020_KI, GM6020_KD,
                    GM6020_OUTPUT_MAX, GM6020_INTEGRAL_MAX);
        }
        for (int i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
            PID_Init(&controller->angle_pids[i], GM6020_ANGLE_KP, GM6020_ANGLE_KI, GM6020_ANGLE_KD,
                    GM6020_ANGLE_OUTPUT_MAX, GM6020_ANGLE_INTEGRAL_MAX);
        }

    void chassis_cmd_calc() //cmd计算函数
{

	    vx_set = rc_ctrl.rc.ch[2] / 200.f;//待矫正
		vy_set = rc_ctrl.rc.ch[3] / 200.f;			
		wz_set = rc_ctrl.rc.ch[4]/200.0f;
        		//运动分解
		chassis_vector_to_M7010_wheel_angle(vx_set, vy_set, wz_set, GM6020_ANGLE);
		chassis_vector_to_M3508_wheel_speed(vx_set, vy_set, wz_set, M3508_SPEED);
			
				//驱动电机 速度环 PID
        
		PID_CurrentLT1 = PID_Calc(&controller->speed_pids[0], controller->motor_feedbacks[0].speed,	M3508_SPEED[0]);
		PID_CurrentRT1 = PID_Calc(&controller->speed_pids[1], controller->motor_feedbacks[1].speed,	M3508_SPEED[1]);
		PID_CurrentLT2 = PID_Calc(&controller->speed_pids[2], controller->motor_feedbacks[2].speed,	M3508_SPEED[2]);
		PID_CurrentRT2 = PID_Calc(&controller->speed_pids[3], controller->motor_feedbacks[3].speed,	M3508_SPEED[3]);			

				//转向电机 角度环串速度环 PID			
		PID_GM6020_Speed_Current_FL = PID_Calc(&controller->GM6020_pids[4], controller->motor_feedbacks[4].angle, GM6020_ANGLE[0], 32767);
		PID_GM6020_Current_FL = PID_Calc(&controller->angle_pids[0], controller->motor_feedbacks[4].speed, PID_GM6020_Speed_Current_FL);

		PID_GM6020_Speed_Current_FR = PID_Calc(&controller->GM6020_pids[5], controller->motor_feedbacks[5].angle, GM6020_ANGLE[1], 32767);
		PID_GM6020_Current_FR = PID_Calc(&controller->angle_pids[1], controller->motor_feedbacks[5].speed, PID_GM6020_Speed_Current_FR);

		}
        CAN_Manager_SendGM6020Current(&hcan1, Chasis_GM6020_ID_1 , PID_GM6020_Current_FL);
        CAN_Manager_SendGM6020Current(&hcan2, Chasis_GM6020_ID_2 , PID_GM6020_Current_FR);
        CAN_Manager_SendMotorCurrents4(&hcan1, MOTOR_STDID_1_4,
            PID_CurrentLT1, PID_CurrentRT1, PID_CurrentLT2, PID_CurrentRT2);

        
    }
		
    void ChassisApp_Init(void) {
        memset(&s_last_cmd, 0, sizeof(s_last_cmd));
        memset(&s_last_sensor, 0, sizeof(s_last_sensor));
        ChassisController_Init(&s_ctrl);
        (void)MsgCenter_Subscribe(TOPIC_CHASSIS_CMD, on_chassis_cmd, NULL);
        (void)MsgCenter_Subscribe(TOPIC_IMU_UPDATE, on_imu_update, NULL);
        (void)MsgCenter_Subscribe(TOPIC_MOTOR_FEEDBACK, on_motor_feedback, NULL);
    }

    ChassisController* ChassisApp_GetController(void) {
        return &s_ctrl;
    }
#endif
