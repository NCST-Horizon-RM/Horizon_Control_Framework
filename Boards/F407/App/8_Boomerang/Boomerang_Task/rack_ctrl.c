//
// Created by lemon on 2026/10/16.
//

#include "rack_ctrl.h"

Rack_State_e rack_state = {0};

void Rack_Task()
{
    if (door_cmd.mode == DOOR_CMD_GAMEOVER)
    {
        Spring_Move();
    }else if (door_cmd.mode == DOOR_CMD_GAMEING)
    {
        if (door_cmd.mode == DOOR_CMD_ROUND1)
        {
            switch (rack_state)
            {
                case FIRE_STATE1:
                    HAL_GPIO_WritePin(GPIOC ,GPIO_PIN_6 ,GPIO_PIN_RESET);
                    Arm_Motor_Increment_Encoder(GIMBAL_MOTOR_MID);
                    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Angle_To_CCR(shoot_ctrl.close_pwm));
                    Spring_Move();
                    if (shoot_ctrl.shoot_state == APPROVE_FIRE)
                    {
                        osDelay(300);
                        Dart_Fire();
                        shoot_ctrl.shoot_state = SHOOT_MOVING;
                        rack_state = FIRE_STATE2;
                    }
                case FIRE_STATE2 :
                    Spring_Move();
                    if (shoot_ctrl.shoot_state == APPROVE_FIRE)
                    {
                        Arm_Servo_Put();
                        osDelay(300);
                        Dart_Fire();
                        Arm_Grab(3);
                        shoot_ctrl.shoot_state = SHOOT_MOVING;
                        rack_state = FIRE_STATE3;
                    }
                case FIRE_STATE3 :
                    Spring_Move();
                    if (shoot_ctrl.shoot_state == APPROVE_FIRE)
                    {
                        Arm_Servo_Put();
                        osDelay(300);
                        Arm_Grab(4);
                        rack_state = FIRE_STATE3_HALF;
                    }
                default:
                    break;
            }
        }else if (door_cmd.mode == DOOR_CMD_ROUND2)
        {
            if (rack_state == FIRE_STATE3_HALF && shoot_ctrl.shoot_state == APPROVE_FIRE)
            {
                osDelay(4000);
                Dart_Fire();
                rack_state = FIRE_STATE4;
            }else if (rack_state == FIRE_STATE4)
            {
                Spring_Move();
                if (shoot_ctrl.shoot_state == APPROVE_FIRE)
                {
                    Arm_Servo_Put();
                    osDelay(300);
                    Dart_Fire();
                    door_cmd.mode = DOOR_CMD_NONE;
                }
            }
        }
    }
}
