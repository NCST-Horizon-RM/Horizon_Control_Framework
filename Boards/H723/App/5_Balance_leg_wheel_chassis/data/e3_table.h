// Auto-generated quartic leg-angle equilibrium offset; do not edit.
#pragma once

static const float e3_coeff[5] = {
    10.83821135f, -17.53636007f, 9.27760300f, -2.13698688f, 0.20151842f
};

static inline float e3_eval(float length) {
    return (((e3_coeff[0] * length + e3_coeff[1]) * length + e3_coeff[2]) * length + e3_coeff[3]) * length + e3_coeff[4];
}
