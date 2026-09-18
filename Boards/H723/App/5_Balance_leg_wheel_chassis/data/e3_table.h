// Auto-generated quartic leg-angle equilibrium offset; do not edit.
#pragma once
#include "k_table_2d.h"

static const float e3_coeff[5] = {
    -25.95248518f, 26.99365680f, -10.63632856f, 2.08213468f, -0.19642741f
};

static inline float e3_eval(float length) {
    return (((e3_coeff[0] * length + e3_coeff[1]) * length + e3_coeff[2]) * length + e3_coeff[3]) * length + e3_coeff[4];
}
