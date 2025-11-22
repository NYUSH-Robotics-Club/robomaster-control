
#pragma once

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "message_center/message_center.h"
#include "shooter_task.h"
#include "chassis_task.h"
#include "gimbal_task.h"

 
osThreadId IMUTaskHandle;
osThreadId robotTaskHandle;
osThreadId motorTaskHandle;


static inline float GetTime_ms(void) {
    return (float)xTaskGetTickCount();
}


void StartIMUTASK(void const *argument);
void StartMOTORTASK(void const *argument);
void StartROBOTTASK(void const *argument);


/**
 * @brief 初始化机器人任务,所有持续运行的任务都在这里初始化
 *
 */
void OSTaskInit()
{
    osThreadDef(instask, StartIMUTASK, osPriorityAboveNormal, 0, 1024);
    IMUTaskHandle = osThreadCreate(osThread(instask), NULL); // 由于是阻塞读取传感器,为姿态解算设置较高优先级,确保以1khz的频率执行
    // // 后续修改为读取传感器数据准备好的中断处理,

    osThreadDef(robotask, StartROBOTTASK, osPriorityNormal, 0, 256);
    robotTaskHandle = osThreadCreate(osThread(robotask), NULL);

    osThreadDef(motortask, StartMOTORTASK, osPriorityNormal, 0, 1024);
    motorTaskHandle = osThreadCreate(osThread(motortask), NULL);



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


__attribute__((noreturn)) void StartROBOTTASK(void const *argument)
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
        ShooterTask();
        ChassisTask();
        GimbalTask();
        ins_dt = GetTime_ms() - ins_start;

        osDelay(1);  // 1ms task period
    }
}


