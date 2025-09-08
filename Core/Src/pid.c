#include "pid.h"

void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float output_max, float integral_max)
{
  pid->Kp = kp;
  pid->Ki = ki;
  pid->Kd = kd;
  pid->output_max = output_max;
  pid->integral_max = integral_max;
  pid->target = 0.0f;
  pid->actual = 0.0f;
  pid->last_actual = 0.0f;
  pid->error = 0.0f;
  pid->last_error = 0.0f;
  pid->integral = 0.0f;
  pid->output = 0.0f;
}

float PID_Calculate(PID_Controller *pid, float target, float actual)
{
  pid->target = target;
  pid->actual = actual;
  pid->error = pid->target - pid->actual;

  pid->integral += pid->error;
  if (pid->integral > pid->integral_max) {
    pid->integral = pid->integral_max;
  } else if (pid->integral < -pid->integral_max) {
    pid->integral = -pid->integral_max;
  }

  float derivative = - (pid->actual - pid->last_actual);
  pid->output = pid->Kp * pid->error + pid->Ki * pid->integral + pid->Kd * derivative;

  if (pid->output > pid->output_max) {
    pid->output = pid->output_max;
  } else if (pid->output < -pid->output_max) {
    pid->output = -pid->output_max;
  }

  pid->last_error = pid->error;
  pid->last_actual = pid->actual;
  return pid->output;
}


