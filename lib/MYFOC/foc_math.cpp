#include "foc_math.h"

#define _1_SQRT3 0.57735026919f
#define _2_SQRT3 1.15470053838f


// ------------------------------------------------------------
// 对应 DengFOC 的 _normalizeAngle()
// 原理：fmod 取余，负数加 2PI
// ------------------------------------------------------------
float normalizeAngle(float angle) {
  float a = fmodf(angle, 2.0f * PI);
  return a >= 0 ? a : (a + 2.0f * PI);
}

// ------------------------------------------------------------
// 对应 DengFOC 的 cal_Iq_Id()
// 原理：
//   Clarke：Iα = Ia，Iβ = (1/√3) * Ia + (2/√3) * Ib
//   Park：  Iq = Iβ * cos(θ) − Iα * sin(θ)
// ------------------------------------------------------------
float calcIq(float ia, float ib, float angle_el) {
    float I_alpha = ia;
    float I_beta  = _1_SQRT3 * ia + _2_SQRT3 * ib;

    float ct = cosf(angle_el);
    float st = sinf(angle_el);

    return I_beta * ct - I_alpha * st;
}