#ifndef BALANCE_LEG_WHEEL_LESO_CONTROL_H
#define BALANCE_LEG_WHEEL_LESO_CONTROL_H

#include <stdbool.h>

#define LESO_STATE_DIM 10
#define LESO_INPUT_DIM 4
#define LESO_AUGMENTED_DIM (LESO_STATE_DIM + LESO_INPUT_DIM)

typedef struct {
    float state[LESO_AUGMENTED_DIM];
    float next_state[LESO_AUGMENTED_DIM];
    float disturbance[LESO_INPUT_DIM];
    float last_delta_input[LESO_INPUT_DIM];
    float ad[LESO_STATE_DIM * LESO_STATE_DIM];
    float bd[LESO_STATE_DIM * LESO_INPUT_DIM];
    float gain[LESO_AUGMENTED_DIM * LESO_STATE_DIM];
    bool initialized;
} LESO_Control_t;

void LESO_Init(LESO_Control_t *leso);
void LESO_Reset(LESO_Control_t *leso);
void LESO_Update(LESO_Control_t *leso, const float measurement[LESO_STATE_DIM],
                 float left_length, float right_length, bool learning_enabled);
void LESO_SetAppliedInput(LESO_Control_t *leso,
                          const float applied_input[LESO_INPUT_DIM],
                          const float equilibrium_input[LESO_INPUT_DIM]);

#endif
