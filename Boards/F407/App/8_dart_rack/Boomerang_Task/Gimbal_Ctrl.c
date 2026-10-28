//
// Created by lemon on 2026/10/11.
//

#include "Gimbal_Ctrl.h"

#define SERVO_NUM 2         //舵机数量

static float Target_Ramp(float now, float target);


static Gimbal_Ctrl_Block_t gimbal_ctrl= {
    .gimbal_num.Servo_Encoder.act_time = 1000,
    .gimbal_num.Servo_Encoder.servo_ids = {1, 2},
    .gimbal_num.Servo_Encoder.wait_encoder = {905, 460},
    .gimbal_num.Servo_Encoder.put1_encoder = {400, 567},
    .gimbal_num.Servo_Encoder.put2_encoder = {140, 700},
    .gimbal_num.Servo_Encoder.dart3_adsorb_encoder = {100, 705},
    .gimbal_num.Servo_Encoder.dart3_remove_encoder = {400, 550},
    .gimbal_num.Servo_Encoder.dart4_adsorb_encoder = {120, 700},
    .gimbal_num.Servo_Encoder.dart4_remove_encoder = {400, 550},
    .gimbal_num.Motor_Encoder.dart3_put_encoder = 3450,
    .gimbal_num.Motor_Encoder.dart3_grab_encoder = 2829,
    .gimbal_num.Motor_Encoder.dart4_put_encoder = 3404,
    .gimbal_num.Motor_Encoder.dart4_grab_encoder = 4092,
};

//机械臂初始化
uint8_t Gimbal_Init()
{
    //电磁铁PWM启动
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

    float PID_TURN_P[3] = {1.0f, 0.01f, 5.0f};
    PID_Init(&gimbal_ctrl.Turn_P, 300.0f, 30.0f, PID_TURN_P,
            0, 0, 0, 0, 0, Integral_Limit | ErrorHandle);
    float PID_TURN_S[3] = {40.0f, 0.0f, 0.0f};
    PID_Init(&gimbal_ctrl.Turn_S, 16000.0f, 10.0f, PID_TURN_S,
            0, 0, 0, 0, 0, Integral_Limit | ErrorHandle);
    //向系统下发云台当前状态，准备中
    System_State_Report(ID_GIMBAL, STATUS_PREPARING);
    return 1;
}

//飞镖抓取函数
void Arm_Grab(uint8_t dart_number)
{
    // 状态汇报
    if (!Is_Group_Online(GIMBAL)) {
        System_State_Report(ID_GIMBAL, STATUS_LOST);
    }
    else{System_State_Report(ID_GIMBAL, STATUS_RUN);}
    // 判断系统状态
    bool is_system_locked = (sys_state.global_mode == GLOBAL_SAFE_LOCK ||
                             sys_state.global_mode == GLOBAL_STANDBY ||
                             sys_state.global_mode == GLOBAL_INIT_STAGE);

    if (gimbal_cmd.mode == GIMBAL_CMD_SAFE || is_system_locked)
    {
        // 清空PID
        for (int i = 0; i < 2; i++) {
            PID_Clear(&gimbal_ctrl.Turn_P);
            PID_Clear(&gimbal_ctrl.Turn_S);

        }
    }
    else
    {
        //3、4发飞镖抓取
        switch (dart_number)
        {
        case 3:
            Arm_Motor_Increment_Encoder(gimbal_ctrl.gimbal_num.Motor_Encoder.dart3_grab_encoder);
            Arm_Servo_Grab(dart_number);
            Arm_Motor_Increment_Encoder(gimbal_ctrl.gimbal_num.Motor_Encoder.dart3_put_encoder);
            break;

        case 4:
            Arm_Motor_Increment_Encoder(gimbal_ctrl.gimbal_num.Motor_Encoder.dart4_grab_encoder);
            Arm_Servo_Grab(dart_number);
            Arm_Motor_Increment_Encoder(gimbal_ctrl.gimbal_num.Motor_Encoder.dart4_put_encoder);
            break;
        default:
            break;
        }
    }
}

//电机增量式移动函数
void Arm_Motor_Increment_Encoder(float target_angle)
{
        while (1)
        {
            // 1. 计算目标值（带缓启动）
            gimbal_ctrl.gimbal_num.Motor_Encoder.increment_motor_encoder =
                Target_Ramp((float)dart_rack_motors.DJI_6020_Turn.Angle_Infinite, target_angle);

            // 2. 延时，给RTOS调度
            osDelay(1);

            // 3. 判断是否到达目标
            if (fabsf(dart_rack_motors.DJI_6020_Turn.Angle_Infinite - target_angle) < 25.0f)
            {
                break;
            }
        }
}

//逐渐增加电机目标值
static float Target_Ramp(float now, float target)
{

    static float out = 0;
    static int init = 0;

    float step = 0.35f;

    if (!init)
    {
        out = now;
        init = 1;
    }

    if (out < target)
    {
        out += step;
        if (out > target) out = target;
    }
    else if (out > target)
    {
        out -= step;
        if (out < target) out = target;
    }

    return out;
}

//舵机抓取动作
void Arm_Servo_Grab(uint8_t dart_number)
{
    ServoMoveMulti(SERVO_NUM,
        gimbal_ctrl.gimbal_num.Servo_Encoder.servo_ids,
        gimbal_ctrl.gimbal_num.Servo_Encoder.wait_encoder,
        gimbal_ctrl.gimbal_num.Servo_Encoder.act_time);
    osDelay(1000);
    switch (dart_number)
    {
        case 3:
            ServoMoveMulti(SERVO_NUM,
                gimbal_ctrl.gimbal_num.Servo_Encoder.servo_ids,
                gimbal_ctrl.gimbal_num.Servo_Encoder.dart3_adsorb_encoder,
                gimbal_ctrl.gimbal_num.Servo_Encoder.act_time);
            osDelay(1000);
            ServoMoveMulti(SERVO_NUM,
                gimbal_ctrl.gimbal_num.Servo_Encoder.servo_ids,
                gimbal_ctrl.gimbal_num.Servo_Encoder.dart3_remove_encoder,
                gimbal_ctrl.gimbal_num.Servo_Encoder.act_time);
            osDelay(1000);
        break;

        case 4:
            ServoMoveMulti(SERVO_NUM,
                gimbal_ctrl.gimbal_num.Servo_Encoder.servo_ids,
                gimbal_ctrl.gimbal_num.Servo_Encoder.dart4_adsorb_encoder,
                gimbal_ctrl.gimbal_num.Servo_Encoder.act_time);
            osDelay(1000);
            ServoMoveMulti(SERVO_NUM,
                gimbal_ctrl.gimbal_num.Servo_Encoder.servo_ids,
                gimbal_ctrl.gimbal_num.Servo_Encoder.dart4_remove_encoder,
                gimbal_ctrl.gimbal_num.Servo_Encoder.act_time);
            osDelay(1000);
        break;
    }
    ServoMoveMulti(SERVO_NUM,
        gimbal_ctrl.gimbal_num.Servo_Encoder.servo_ids,
        gimbal_ctrl.gimbal_num.Servo_Encoder.wait_encoder,
        gimbal_ctrl.gimbal_num.Servo_Encoder.act_time);
    osDelay(1000);
}

//舵机放置动作
void Arm_Servo_Put()
{
    ServoMoveMulti(SERVO_NUM,
        gimbal_ctrl.gimbal_num.Servo_Encoder.servo_ids,
        gimbal_ctrl.gimbal_num.Servo_Encoder.wait_encoder,
        gimbal_ctrl.gimbal_num.Servo_Encoder.act_time);
    osDelay(1000);
    ServoMoveMulti(SERVO_NUM,
        gimbal_ctrl.gimbal_num.Servo_Encoder.servo_ids,
        gimbal_ctrl.gimbal_num.Servo_Encoder.put1_encoder,
        gimbal_ctrl.gimbal_num.Servo_Encoder.act_time);
    osDelay(1000);
    ServoMoveMulti(SERVO_NUM,
        gimbal_ctrl.gimbal_num.Servo_Encoder.servo_ids,
        gimbal_ctrl.gimbal_num.Servo_Encoder.put2_encoder,
        gimbal_ctrl.gimbal_num.Servo_Encoder.act_time);
    osDelay(1000);
    HAL_GPIO_WritePin(GPIOB ,GPIO_PIN_12 ,GPIO_PIN_SET);//电磁铁下磁
    ServoMoveMulti(SERVO_NUM,
        gimbal_ctrl.gimbal_num.Servo_Encoder.servo_ids,
        gimbal_ctrl.gimbal_num.Servo_Encoder.wait_encoder,
        gimbal_ctrl.gimbal_num.Servo_Encoder.act_time);
    osDelay(1000);
    HAL_GPIO_WritePin(GPIOB ,GPIO_PIN_12 ,GPIO_PIN_RESET);//电磁铁重新上磁
}

void Arm_Motor_PID()
{
    PID_Calculate(&gimbal_ctrl.Turn_P,dart_rack_motors.DJI_6020_Turn.Angle_Infinite,gimbal_ctrl.gimbal_num.Motor_Encoder.increment_motor_encoder);
    PID_Calculate(&gimbal_ctrl.Turn_S,dart_rack_motors.DJI_6020_Turn.Speed_now,gimbal_ctrl.Turn_P.Output);

    DJI_Motor_Send(&hcan2,0x1FE,(int16_t)gimbal_ctrl.Turn_S.Output,0,0,0);
}