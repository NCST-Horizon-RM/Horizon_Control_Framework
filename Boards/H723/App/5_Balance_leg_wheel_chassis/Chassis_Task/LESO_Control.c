#include "LESO_Control.h"
#include "leso_table_2d.h"
#include <math.h>
#include <string.h>

#define LESO_DISTURBANCE_DECAY 0.995f

static const float disturbance_limit[LESO_INPUT_DIM] = {
    20.0f, 20.0f, 4.0f, 4.0f
};

static float clamp_symmetric(float value, float limit)
{
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

void LESO_Init(LESO_Control_t *leso)
{
    LESO_Reset(leso);
}

void LESO_Reset(LESO_Control_t *leso)
{
    if (leso == NULL) {
        return;
    }
    memset(leso, 0, sizeof(*leso));
}

void LESO_Update(LESO_Control_t *leso, const float measurement[LESO_STATE_DIM],
                 float left_length, float right_length, bool learning_enabled)
{
    if (leso == NULL || measurement == NULL) {
        return;
    }

    if (!leso->initialized) {
        memcpy(leso->state, measurement, LESO_STATE_DIM * sizeof(float));
        leso->initialized = true;
        return;
    }

    if (!learning_enabled) {
        memcpy(leso->state, measurement, LESO_STATE_DIM * sizeof(float));
        for (int input = 0; input < LESO_INPUT_DIM; input++) {
            leso->state[LESO_STATE_DIM + input] *= LESO_DISTURBANCE_DECAY;
            leso->disturbance[input] = leso->state[LESO_STATE_DIM + input];
        }
        return;
    }

    leso_table_2d_eval(left_length, right_length,
                       leso->ad, leso->bd, leso->gain);

    float innovation[LESO_STATE_DIM];
    for (int state = 0; state < LESO_STATE_DIM; state++) {
        innovation[state] = measurement[state] - leso->state[state];
    }

    for (int row = 0; row < LESO_STATE_DIM; row++) {
        float value = 0.0f;
        for (int column = 0; column < LESO_STATE_DIM; column++) {
            value += leso->ad[row * LESO_STATE_DIM + column]
                   * leso->state[column];
        }
        for (int input = 0; input < LESO_INPUT_DIM; input++) {
            value += leso->bd[row * LESO_INPUT_DIM + input]
                   * (leso->last_delta_input[input]
                      + leso->state[LESO_STATE_DIM + input]);
        }
        for (int output = 0; output < LESO_STATE_DIM; output++) {
            value += leso->gain[row * LESO_STATE_DIM + output]
                   * innovation[output];
        }
        leso->next_state[row] = value;
    }

    for (int disturbance = 0; disturbance < LESO_INPUT_DIM; disturbance++) {
        const int row = LESO_STATE_DIM + disturbance;
        float value = leso->state[row];
        for (int output = 0; output < LESO_STATE_DIM; output++) {
            value += leso->gain[row * LESO_STATE_DIM + output]
                   * innovation[output];
        }
        leso->next_state[row] = clamp_symmetric(
            value, disturbance_limit[disturbance]
        );
    }

    for (int state = 0; state < LESO_AUGMENTED_DIM; state++) {
        if (!isfinite(leso->next_state[state])) {
            LESO_Reset(leso);
            memcpy(leso->state, measurement, LESO_STATE_DIM * sizeof(float));
            leso->initialized = true;
            return;
        }
    }

    memcpy(leso->state, leso->next_state, sizeof(leso->state));
    memcpy(leso->disturbance, &leso->state[LESO_STATE_DIM],
           sizeof(leso->disturbance));
}

void LESO_SetAppliedInput(LESO_Control_t *leso,
                          const float applied_input[LESO_INPUT_DIM],
                          const float equilibrium_input[LESO_INPUT_DIM])
{
    if (leso == NULL || applied_input == NULL || equilibrium_input == NULL) {
        return;
    }
    for (int input = 0; input < LESO_INPUT_DIM; input++) {
        leso->last_delta_input[input] =
            applied_input[input] - equilibrium_input[input];
    }
}
