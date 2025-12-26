/**
 * @file controller.c
 * @author (Reference:Hunan University open-source code 2022)
 */
/* ----------------------------pid input example---------------------------- */
/**
 * PID_Init_Config_s yaw_angle_loop_init = {
    .Kp = 1.0f,
    .Ki = 0.5f,
    .Kd = 0.1f,
    .MaxOut = 1000.0f,
    .DeadBand = 0.01f,
    .Improve = PID_Integral_Limit | PID_Derivative_On_Measurement | PID_OutputFilter,
    .IntegralLimit = 300.0f,
    .CoefA = 0,
    .CoefB = 0,
    .Output_LPF_RC = 0.01f,
    .Derivative_LPF_RC = 0.01f
};
 output=PIDCalculate(PIDInstance *pid, float measure, float ref)
 */
#include "pid.h"
#include "memory.h"
#include <math.h>
#include "stdlib.h"



extern float DWT_GetDeltaT(void *cnt);

/* ----------------------------pid improvement---------------------------- */

static void improve_Trapezoid_Intergral(PIDInstance *pid)
{
    // calculate the area of trapezoid, a smmother way to calculate integral
    pid->ITerm = pid->Ki * ((pid->Err + pid->Last_Err) / 2) * pid->dt;
}

// automatically change the integration rate based on error magnitude
static void improve_changing_Integration_Rate(PIDInstance *pid)
{
    if (pid->Err * pid->Iout > 0)// The Integration is helping
    {
        
        if (fabsf(pid->Err) <= pid->CoefB)
            return; // small error, integral fully enabled
        if (fabsf(pid->Err) <= (pid->CoefA + pid->CoefB))
            pid->ITerm *= (pid->CoefA - fabsf(pid->Err) + pid->CoefB) / pid->CoefA;//medium error, decreasing integral
        else // large error, disable integral
            pid->ITerm = 0;
    }
}

static void improve_Integral_Limit(PIDInstance *pid)//cannot be used at first
{
    static float intended_Output, intended_Iout;
    intended_Iout = pid->Iout + pid->ITerm;
    intended_Output = pid->Pout + pid->Iout + pid->Dout;
    if (fabsf(intended_Output) > pid->MaxOut)
    {
        if (pid->Err * pid->Iout > 0) // Integration is increasing
        {
            pid->ITerm = 0; // disable integral
        }
    }

    if (intended_Iout > pid->IntegralLimit)
    {
        pid->ITerm = 0;
        pid->Iout = pid->IntegralLimit;
    }
    if (intended_Iout < -pid->IntegralLimit)
    {
        pid->ITerm = 0;
        pid->Iout = -pid->IntegralLimit;
    }
}

// 微分先行 Using Measurement to calculate derivative rather than error
static void improve_Derivative_On_Measurement(PIDInstance *pid)
{
    pid->Dout = pid->Kd * (pid->Last_Measure - pid->Measure) / pid->dt;
}

// 微分滤波(采集微分时,滤除高频噪声) filt the noise when get derivative
static void improve_Derivative_Filter(PIDInstance *pid)
{
    pid->Dout = pid->Dout * pid->dt / (pid->Derivative_LPF_RC + pid->dt) +
                pid->Last_Dout * pid->Derivative_LPF_RC / (pid->Derivative_LPF_RC + pid->dt);
                //一阶低通滤波器 减小高频抖动
}

// The same method as Derivative filter
static void improve_Output_Filter(PIDInstance *pid)
{
    pid->Output = pid->Output * pid->dt / (pid->Output_LPF_RC + pid->dt) +
                  pid->Last_Output * pid->Output_LPF_RC / (pid->Output_LPF_RC + pid->dt);
}


static void f_Output_Limit(PIDInstance *pid)
{
    if (pid->Output > pid->MaxOut)
    {
        pid->Output = pid->MaxOut;
    }
    if (pid->Output < -(pid->MaxOut))
    {
        pid->Output = -(pid->MaxOut);
    }
}

// 电机堵转检测
// static void f_PID_ErrorHandle(PIDInstance *pid)
// {
    // /*Motor Blocked Handle*/
    // if (fabsf(pid->Output) < pid->MaxOut * 0.001f || fabsf(pid->Ref) < 0.0001f)
        // return;
// 
    // if ((fabsf(pid->Ref - pid->Measure) / fabsf(pid->Ref)) > 0.95f)
    // {
        // Motor blocked counting
        // pid->ERRORHandler.ERRORCount++;
    // }
    // else
    // {
        // pid->ERRORHandler.ERRORCount = 0;
    // }
// 
    // if (pid->ERRORHandler.ERRORCount > 500)
    // {
        // Motor blocked over 1000times
        // pid->ERRORHandler.ERRORType = PID_MOTOR_BLOCKED_ERROR;
    // }
// }

void PIDInit(PIDInstance *pid, PID_Init_Config *config)
{
    // config的数据和pid的部分数据是连续且相同的的,所以可以直接用memcpy

    memset(pid, 0, sizeof(PIDInstance));
    // utilize the quality of struct that its memeory is continuous
    memcpy(pid, config, sizeof(PID_Init_Config));
    // set rest of memory to 0
    DWT_GetDeltaT(&pid->DWT_CNT);
}
 */
float PIDCalculate(PIDInstance *pid, float measure, float ref)
{
    // 堵转检测
    // if (pid->Improve & PID_ErrorHandle)
        // f_PID_ErrorHandle(pid);

     pid->dt = DWT_GetDeltaT(&pid->DWT_CNT); // 获取两次pid计算的时间间隔,用于积分和微分

   
    pid->Measure = measure;
    pid->Ref = ref;
    pid->Err = pid->Ref - pid->Measure;

    
    if (abs(pid->Err) > pid->DeadBand)
    {
      
        pid->Pout = pid->Kp * pid->Err;
        pid->ITerm = pid->Ki * pid->Err * pid->dt;
        pid->Dout = pid->Kd * (pid->Err - pid->Last_Err) / pid->dt;

        
        if (pid->Improve & PID_Trapezoid_Intergral)
            f_Trapezoid_Intergral(pid);
        
        if (pid->Improve & PID_ChangingIntegrationRate)
            f_Changing_Integration_Rate(pid);
        
        if (pid->Improve & PID_Derivative_On_Measurement)
            f_Derivative_On_Measurement(pid);
        
        if (pid->Improve & PID_DerivativeFilter)
            f_Derivative_Filter(pid);
        
        if (pid->Improve & PID_Integral_Limit)
            f_Integral_Limit(pid);

        pid->Iout += pid->ITerm;                         
        pid->Output = pid->Pout + pid->Iout + pid->Dout; 

        // filt
        if (pid->Improve & PID_OutputFilter)
            f_Output_Filter(pid);

        // limit
        f_Output_Limit(pid);
    }
    else // if in the deadzone
    {
        pid->Output = 0;
        pid->ITerm = 0;
    }

    // save the data
    pid->Last_Measure = pid->Measure;
    pid->Last_Output = pid->Output;
    pid->Last_Dout = pid->Dout;
    pid->Last_Err = pid->Err;
    pid->Last_ITerm = pid->ITerm;

    return pid->Output;
}