#include "dm10010l_motor.h"   // project motor types
#include "pid.h"              // PID helpers
#include "can.h"              // CAN handle defs
#include "can_manager.h"      // CAN send helpers
#include "message_center.h"  // message subscription
#include "can_comm.h"        // CAN parsing types/events
// #include "printing.h"        // debug printing (optional)
#include <string.h>
#include <stdlib.h>
#include "cmsis_os_compat.h" // lightweight compat for os* symbols


static uint8_t idx;
static DMMotorInstance *dm_motor_instance[DM_MOTOR_CNT];
static osThreadId dm_task_handle[DM_MOTOR_CNT];

/* Message handler for DM10010L feedback published by can_comm */
static void on_dm10010l_feedback(const MsgEvent *ev, void *user)
{
    (void)user;
    if (ev->size != sizeof(DM10010LFeedbackEvent)) return;
    const DM10010LFeedbackEvent *m = (const DM10010LFeedbackEvent *)ev->data;
    if (m->motor_id < 1 || m->motor_id > DM_MOTOR_CNT) return;
    /* motor_id in DM10010LFeedbackEvent is 1..7; our array indexed 0.. */
    uint8_t idx0 = m->motor_id - 1;
    if (idx0 >= idx) return; // not registered
    DMMotorInstance *motor = dm_motor_instance[idx0];
    if (!motor) return;
    motor->measure.last_position = motor->measure.position;
    motor->measure.position = (float)m->pos / 1000.0f;
    motor->measure.velocity = (float)m->vel / 1000.0f;
    motor->measure.torque = (float)m->torque / 1000.0f;
    motor->measure.T_Mos = (float)m->t_mos;
    motor->measure.T_Rotor = (float)m->t_rotor;
}
/* 两个用于将uint值和float值进行映射的函数,在设定发送值和解析反馈值时使用 */
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

static void DMMotorSetMode(DMMotor_Mode_e cmd, DMMotorInstance *motor)
{
    memset(motor->motor_can_instace->tx_buff, 0xff, 7);  // 发送电机指令的时候前面7bytes都是0xff
    motor->motor_can_instace->tx_buff[7] = (uint8_t)cmd; // 最后一位是命令id
    CANTransmit(motor->motor_can_instace, 1);
}

static void DMMotorDecode(CANInstance *motor_can)
{
    uint16_t tmp; // 用于暂存解析值,稍后转换成float数据,避免多次创建临时变量
    uint8_t *rxbuff = motor_can->rx_buff;
    DMMotorInstance *motor = (DMMotorInstance *)motor_can->id;
    DM_Motor_Measure_s *measure = &(motor->measure); // 将can实例中保存的id转换成电机实例的指针

    DaemonReload(motor->motor_daemon);

    measure->last_position = measure->position;
    tmp = (uint16_t)((rxbuff[1] << 8) | rxbuff[2]);
    measure->position = uint_to_float(tmp, DM_P_MIN, DM_P_MAX, 16);

    tmp = (uint16_t)((rxbuff[3] << 4) | rxbuff[4] >> 4);
    measure->velocity = uint_to_float(tmp, DM_V_MIN, DM_V_MAX, 12);

    tmp = (uint16_t)(((rxbuff[4] & 0x0f) << 8) | rxbuff[5]);
    measure->torque = uint_to_float(tmp, DM_T_MIN, DM_T_MAX, 12);

    measure->T_Mos = (float)rxbuff[6];
    measure->T_Rotor = (float)rxbuff[7];
}

static void DMMotorLostCallback(void *motor_ptr)
{
}
void DMMotorCaliEncoder(DMMotorInstance *motor)
{
    DMMotorSetMode(DM_CMD_ZERO_POSITION, motor);
    DWT_Delay(0.1);
}
DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config)
{
     /* Create instance and set sensible defaults so module compiles and runs
         in this project's environment. Original HNU-specific config is optional. */
     DMMotorInstance *motor = (DMMotorInstance *)malloc(sizeof(DMMotorInstance));
     if (!motor) return NULL;
     memset(motor, 0, sizeof(DMMotorInstance));

     /* assign incremental id (1..DM_MOTOR_CNT). Integration code may set ids
         differently if needed. */
     motor->id = (uint8_t)(idx + 1);

     /* Initialize PID instances with conservative defaults. If the original
         platform provides PID parameters via `config`, integration code can
         be extended to copy those values. */
     PID_Init(&motor->current_PID, 0.0f, 0.0f, 0.0f, 1000.0f, 100.0f);
     PID_Reset(&motor->current_PID);
     PID_Init(&motor->speed_PID, 0.0f, 0.0f, 0.0f, 30000.0f, 4000.0f);
     PID_Reset(&motor->speed_PID);
     PID_Init(&motor->angle_PID, 0.0f, 0.0f, 0.0f, 30000.0f, 4000.0f);
     PID_Reset(&motor->angle_PID);

     /* Register to receive parsed DM10010L feedback events published by
         the project's CAN layer. */
     (void)MsgCenter_Subscribe(TOPIC_DM10010L_FEEDBACK, on_dm10010l_feedback, NULL);

     /* store instance and return */
     dm_motor_instance[idx++] = motor;
     return motor;
}

void DMMotorSetRef(DMMotorInstance *motor, float ref)
{
    motor->pid_ref = ref;
}

void DMMotorEnable(DMMotorInstance *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
}

void DMMotorStop(DMMotorInstance *motor)//不使用使能模式是因为需要收到反馈
{
    motor->stop_flag = MOTOR_STOP;
}

void DMMotorOuterLoop(DMMotorInstance *motor, int closeloop_type)
{
    motor->motor_settings.outer_loop_type = closeloop_type;
}


//@Todo: 目前只实现了力控，更多位控PID等请自行添加
void DMMotorTask(void const *argument)
{
    float  pid_ref, set;
    DMMotorInstance *motor = (DMMotorInstance *)argument;
   //DM_Motor_Measure_s *measure = &motor->measure;
    Motor_Control_Setting_s *setting = &motor->motor_settings;
    //CANInstance *motor_can = motor->motor_can_instace;
    //uint16_t tmp;
    DMMotor_Send_s motor_send_mailbox;
    while (1)
    {
        pid_ref = motor->pid_ref;
        
        set = pid_ref;
        if (setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
            set *= -1;
       
        LIMIT_MIN_MAX(set, DM_T_MIN, DM_T_MAX);
          /* For project integration we send position/velocity commands via
              CAN manager. Original HNU code packed torque/current into bytes;
              here we'll send POS/VEL (torque control not supported by the
              project's CAN helper in this codepath). */
          motor_send_mailbox.position_des = float_to_uint(0, DM_P_MIN, DM_P_MAX, 16);
          motor_send_mailbox.velocity_des = float_to_uint(0, DM_V_MIN, DM_V_MAX, 12);
          motor_send_mailbox.torque_des = float_to_uint(pid_ref, DM_T_MIN, DM_T_MAX, 12);
        motor_send_mailbox.Kp = 0;
        motor_send_mailbox.Kd = 0;

        if(motor->stop_flag == MOTOR_STOP)
            motor_send_mailbox.torque_des = float_to_uint(0, DM_T_MIN, DM_T_MAX, 12);

          /* Use CAN manager helper. We pass desired position and velocity as
              floats; the helper will pack them appropriately. Torque mode is
              not used here. */
          (void)CAN_Manager_SendDM10010LPOSVES(&hcan1, motor->id, 0.0f, 0.0f);

        osDelay(2);
    }
}
void DMMotorControlInit()
{
    char dm_task_name[5] = "dm";
    // 遍历所有电机实例,创建任务
    if (!idx)
        return;
    for (size_t i = 0; i < idx; i++)
    {
        char dm_id_buff[2] = {0};
        __itoa(i, dm_id_buff, 10);
        strcat(dm_task_name, dm_id_buff);
        osThreadDef(dm_task_name, DMMotorTask, osPriorityNormal, 0, 128);
        dm_task_handle[i] = osThreadCreate(osThread(dm_task_name), dm_motor_instance[i]);
    }
}