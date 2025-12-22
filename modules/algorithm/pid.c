#include "pid.h"
#include <stddef.h>
/**
 * @brief Initialize PID controller with specified parameters
 * @param pid Pointer to PID controller structure
 * @param kp Proportional gain
 * @param ki Integral gain
 * @param kd Derivative gain
 * @param output_max Maximum output value (saturation limit)
 * @param integral_max Maximum integral value (anti-windup limit)
 */
void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float output_max, float integral_max)
{
  pid->Kp = kp;
  pid->Ki = ki;
  pid->Kd = kd;
  
  pid->output_max = output_max;
  pid->integral_max = integral_max;
  
  pid->target = 0.0f;
  pid->error[0] = 0.0f;
  pid->error[1] = 0.0f;
  pid->error[2] = 0.0f;
  pid->integral = 0.0f;
  pid->output = 0.0f;
}

/**
 * @brief Calculate PID controller output
 * @param pid Pointer to PID controller structure
 * @param target Target value (setpoint)
 * @param actual Current actual value (feedback)
 * @return PID controller output
 */
float PID_Calculate(PID_Controller *pid, float target, float actual)
{
  pid->target = target;
  pid->actual = actual;
  
  pid->error[2] = pid->error[1];
  pid->error[1] = pid->error[0];
  pid->error[0] = pid->target - pid->actual;
  

  pid->integral += pid->error[0];
  if (pid->integral > pid->integral_max) {
    pid->integral = pid->integral_max;
  } else if (pid->integral < -pid->integral_max) {
    pid->integral = -pid->integral_max;
  }

  float derivative = (pid->error[0] - pid->error[1]);
  
  pid->output = pid->Kp * pid->error[0] + pid->Ki * pid->integral + pid->Kd * derivative;

  if (pid->output > pid->output_max) {
    pid->output = pid->output_max;
  } else if (pid->output < -pid->output_max) {
    pid->output = -pid->output_max;
  }


  
  return pid->output;
}


float PID_RPM_Calculate(PID_Controller *pid, float target_rpm, float actual_rpm)
{
    pid->target = target_rpm;
    pid->actual = actual_rpm;

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->error[0] = pid->target - pid->actual;

    pid->integral += pid->Ki * pid->error[0];
    if (pid->integral > pid->integral_max) {
        pid->integral = pid->integral_max;
    } else if (pid->integral < -pid->integral_max) {
        pid->integral = -pid->integral_max;
    }

    float derivative = - (pid->error[0]- pid->error[1]) * pid->Kd;

    pid->output = pid->Kp * pid->error[0] + pid->integral + derivative;

    if (pid->output > pid->output_max) {
        pid->output = pid->output_max;
    } else if (pid->output < -pid->output_max) {
        pid->output = -pid->output_max;
    }

    return pid->output;
}

void PID_Reset(PID_Controller *pid)
{
    if (pid == NULL) return;
    pid->integral = 0.0f;
    pid->error[0] = 0.0f;
    pid->error[1] = 0.0f;
    pid->error[2] = 0.0f;
    pid->output = 0.0f;
}


