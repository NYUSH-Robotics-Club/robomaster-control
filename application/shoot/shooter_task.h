#include "shooter_controller.h"

void ShooterTask(){
    ShooterController_Update();
    ShooterController_ComputeCurrents();
}