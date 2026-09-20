// Auto-generated quartic leg-angle equilibrium offset; do not edit.
#pragma once

static const float e3_coeff[5] = {
    48.26729184f, -49.80068573f, 19.07902575f, -3.48980823f, 0.28193981f
};

static inline float e3_eval(float length) {
    return (((e3_coeff[0] * length + e3_coeff[1]) * length + e3_coeff[2]) * length + e3_coeff[3]) * length + e3_coeff[4];
}
