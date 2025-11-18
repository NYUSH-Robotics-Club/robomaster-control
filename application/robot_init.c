#include "shooter_controller.h"
#include "cmd_controller.h"
#include "message_center.h"
#include "gimbal_controller.h"
#include "chassis_controller.h"
#include "can_manager.h"
#include "remote_control.h"
#include "vision_comm.h"

void Robot_Init(void) {
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