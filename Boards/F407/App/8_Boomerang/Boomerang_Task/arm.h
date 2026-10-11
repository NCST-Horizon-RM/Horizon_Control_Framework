//
// Created by lemon on 2026/10/11.
//

#ifndef HORIZON_CONTROL_FRAMEWORK_ARM_H
#define HORIZON_CONTROL_FRAMEWORK_ARM_H

#include <stdint.h>
#include <math.h>
#include "cmsis_os.h"
#include "Robot_Config.h"

typedef struct
{
    uint16_t wait_encoder[2];
    uint16_t put1_encoder[2];
    uint16_t put2_encoder[2];
    uint16_t dart3_adsorb_encoder[2];
    uint16_t dart3_remove_encoder[2];
    uint16_t dart4_adsorb_encoder[2];
    uint16_t dart4_remove_encoder[2];
}Grab_Encoder;

typedef struct
{
    int16_t dart3;
    int16_t dart4;
}Middle_Encoder;

void Motor_TurnTo_Angle(float target_angle);
void Arm_Action_Sequence(float grap_angle);
void Dart_Grab_Action_3();
void Dart_Grab_Action_4();
void Dart_put();

#endif //HORIZON_CONTROL_FRAMEWORK_ARM_H
