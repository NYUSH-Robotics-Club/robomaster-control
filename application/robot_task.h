
#pragma once

#include "FreeRTOS.h"
#include "main.h"
#include


osThreadId IMUTaskHandle;
osThreadId robotTaskHandle;
osThreadId motorTaskHandle;
osThreadId daemonTaskHandle;
osThreadId uiTaskHandle;

static inline float GetTime_ms(void) {
    return (float)osKernelGetTickCount();
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

    LOGINFO("[freeRTOS] INS Task Start");
    for (;;)
    {
        ins_start = GetTime_ms();

        gyro_data_update();

        ins_dt = GetTime_ms() - ins_start;
        if (ins_dt > 1.0f)
            LOGERROR("[freeRTOS] INS Task is being DELAY! dt = [%f]", ins_dt);

        osDelay(1);  // 1ms task period
    }
}


