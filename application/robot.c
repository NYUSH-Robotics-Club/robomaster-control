#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdarg.h>

#include "bmi088driver.h"
#include "wt61c.h"
#include "gyro_data.h"
#include "buzzer.h"
#include "usbd_cdc_if.h"
#include "printing.h"

#include "can_manager.h"
#include "chassis_controller.h"
#include "gimbal_controller.h"
#include "shooter_controller.h"
#include "cmd_controller.h"
#include "message_center.h"
#include "app_subscriptions.h"
#include "remote_control.h"
#include "vision_comm.h"
#include "robot_task.h"
CAN_Manager_t can1_manager;
CAN_Manager_t can2_manager;


void Robot_Init(void) {
    BMI088_init();
    MsgCenter_Init();

    // Initialize command controller first (central control)
    CmdController_Init();
    // Initialize application controllers
    
    ChassisApp_Init();
    ShooterApp_Init();
    GimbalApp_Init();

    // Initialize remote control
    remote_control_init();

    CAN_Manager_Init(&can1_manager, CAN_CHANNEL_1, &hcan1);
    CAN_Manager_Init(&can2_manager, CAN_CHANNEL_2, &hcan2);
    CAN_Manager_Start(&can1_manager);
    CAN_Manager_Start(&can2_manager);

    VisionComm_Init();

}

void Robot_task(void){
    
}