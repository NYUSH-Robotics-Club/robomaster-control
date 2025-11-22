#include "chassis_controller.h"

void ChassisTask(){
    ChassisController_Update();
    ChassisController_ComputeCurrents();
}