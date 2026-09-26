// Auto-generated quartic leg-angle equilibrium offset; do not edit.
#pragma once

static const float e3_coeff[5] = {
    12.23361804f, -19.55147261f, 10.28173426f, -2.35944672f, 0.22192577f
};

static inline float e3_eval(float length) {
    return (((e3_coeff[0] * length + e3_coeff[1]) * length + e3_coeff[2]) * length + e3_coeff[3]) * length + e3_coeff[4];
}
