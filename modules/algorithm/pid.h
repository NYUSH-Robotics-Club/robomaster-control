/**
 * @brief Advanced PID controller algorithim
 * @file pid.h
 * @author (Reference:Hunan University open-source code 2022)
 */
#ifndef PID_H
#define PID_H

#include <stdint.h>
#include <math.h>
#include <memory.h>

/* Avoid defining a global macro named 'abs' which can conflict with C++ overloads/templates.
   Provide a local inline function for floating-point absolute value instead. */
static inline float pid_absf(float x) { return (x >= 0.0f) ? x : -x; }

// Improvement swithces for PID controller
typedef enum
{
    PID_IMPROVE_NONE = 0b00000000,                // 0000 0000
    PID_Integral_Limit = 0b00000001,              // 0000 0001
    PID_Derivative_On_Measurement = 0b00000010,   // 0000 0010
    PID_Trapezoid_Intergral = 0b00000100,         // 0000 0100
    PID_ProportionalOnMeasurement = 0b00001000, // 0000 1000
    PID_OutputFilter = 0b00010000,                // 0001 0000
    PID_ChangingIntegrationRate = 0b00100000,     // 0010 0000
    PID_DerivativeFilter = 0b01000000,            // 0100 0000
    PID_ErrorHandle = 0b10000000,                 // 1000 0000
} PID_Improvement;

//typedef enum errorType_e
// {
 // PID_ERROR_NONE = 0x00U,
 // PID_MOTOR_BLOCKED_ERROR = 0x01U
// } ErrorType_e;

// typedef struct
// {
    // uint64_t ERRORCount;
    // ErrorType_e ERRORType;
// } PID_ErrorHandler_t;



typedef struct
{
    //---------------------------------- init config block
    // config parameter
    float Kp;
    float Ki;
    float Kd;
    float MaxOut;
    float DeadBand;

    // improve parameter
    PID_Improvement Improve;
    float IntegralLimit;     // 积分限幅
    float CoefA;             // 变速积分 For Changing Integral
    float CoefB;             // 变速积分 ITerm = Err*((A-abs(err)+B)/A)  when B<|err|<A+B
    float Output_LPF_RC;     // 输出滤波器 RC = 1/omegac
    float Derivative_LPF_RC; // 微分滤波器系数

    //-----------------------------------
    // for calculating
    float Measure;
    float Last_Measure;
    float Err;
    float Last_Err;
    float Last_ITerm;

    float Pout;
    float Iout;
    float Dout;
    float ITerm;

    float Output;
    float Last_Output;
    float Last_Dout;

    float Ref;

    uint32_t DWT_CNT;
    float dt;

    //PID_ErrorHandler_t ERRORHandler;
} PIDInstance;

/* Struct for PID initialization */
typedef struct // config parameter
{
    // basic parameter
    float Kp;
    float Ki;
    float Kd;
    float MaxOut;   
    float DeadBand; // Dead band

    // improve parameter
    PID_Improvement Improve;
    float Integral_Limit; 
    float CoefA;         // AB为变速积分参数,变速积分实际上就引入了积分分离
    float CoefB;         // ITerm = Err*((A-abs(err)+B)/A)  when B<|err|<A+B
    float Output_LPF_RC; // RC = 1/omegac
    float Derivative_LPF_RC;
} PID_Init_Config;

/**
 * @brief pid init
 * */
void PIDInit(PIDInstance *pid, PID_Init_Config *config);

/**
 * @brief 计算PID输出
 *
 * @param pid     PID实例指针
 * @param measure 反馈值
 * @param ref     设定值
 * @return float  PID计算输出
 */
float PIDCalculate(PIDInstance *pid, float measure, float ref);

#endif // PID_H

