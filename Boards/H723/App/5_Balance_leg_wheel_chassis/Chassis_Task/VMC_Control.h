#ifndef BALANCE_LEG_WHEEL_VMC_CONTROL_H
#define BALANCE_LEG_WHEEL_VMC_CONTROL_H

#include <stdbool.h>

#define VMC_L1_LENGTH 0.215f
#define VMC_L2_LENGTH 0.258f
#define VMC_L3_LENGTH 0.258f
#define VMC_L4_LENGTH 0.215f
#define VMC_THETA_RATE_LPF_HZ 25.0f

typedef struct {
    float length[3];
    float phi;
    float theta[3];
    float length_last[2];
    float theta_last[2];
    float Tp_front;
    float Tp_back;
} VMC_LegSite_t;

typedef struct {
    float l1, l2, l3, l4;
    float JRM_l[2][2];
    float JRM_r[2][2];
    VMC_LegSite_t left;
    VMC_LegSite_t right;
    bool derivative_initialized;
} VMC_Control_t;

void VMC_Init(VMC_Control_t *vmc, float l1, float l2, float l3, float l4);
void VMC_Update(VMC_Control_t *vmc, float dt, float body_pitch_rad,
                float pos_front_L, float pos_back_L,
                float pos_front_R, float pos_back_R);
void VMC_ForceToTorque(const float JRM[2][2], float force, float Tp_target,VMC_LegSite_t *leg);

#endif
