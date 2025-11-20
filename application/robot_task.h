
#pragma once

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "message_center/message_center.h"

 
osThreadId IMUTaskHandle;
osThreadId robotTaskHandle;
osThreadId motorTaskHandle;
osThreadId daemonTaskHandle;

static inline float GetTime_ms(void) {
    return (float)xTaskGetTickCount();
}


void StartIMUTASK(void const *argument);
void StartMOTORTASK(void const *argument);
void StartRobotCMD(void const *argument);
void StartGimbalTask(void const *argument);

/**
 * @brief 初始化机器人任务,所有持续运行的任务都在这里初始化
 *
 */
void OSTaskInit()
{
    osThreadDef(instask, StartIMUTASK, osPriorityAboveNormal, 0, 1024);
    IMUTaskHandle = osThreadCreate(osThread(instask), NULL); // 由于是阻塞读取传感器,为姿态解算设置较高优先级,确保以1khz的频率执行
    // // 后续修改为读取传感器数据准备好的中断处理,



}

__attribute__((noreturn)) void StartIMUTASK(void const *argument)
{
    static float ins_start;
    static float ins_dt;

    for (;;)
    {
        ins_start = GetTime_ms();

        gyro_data_update();

        ins_dt = GetTime_ms() - ins_start;

        osDelay(1);  // 1ms task period
    }
}


__attribute__((noreturn)) void StartRobotCMD(void const *argument)
{
    static float ins_start;
    static float ins_dt;

    for (;;)
    {
        ins_start = GetTime_ms();
        CmdController_Task((uint32_t)ins_start);
        MsgCenter_Dispatch();

        ins_dt = GetTime_ms() - ins_start;

        osDelay(1);  // 1ms task period
    }
}

__attribute__((noreturn)) void StartMOTORTASK(void const *argument)
{
    static float ins_start;
    static float ins_dt;

    for (;;)
    {
        ins_start = GetTime_ms();
        ChassisController_ComputeCurrents();
        ShooterController_ComputeCurrents();
        ins_dt = GetTime_ms() - ins_start;

        osDelay(1);  // 1ms task period
    }
}
