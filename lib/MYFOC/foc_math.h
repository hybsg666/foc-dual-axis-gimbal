#ifndef FOC_MATH_H
#define FOC_MATH_H

#include <Arduino.h>

// ------------------------------------------------------------
// 角度归一化到 [0, 2PI]
// 对应 DengFOC 的 _normalizeAngle()
// 输入任意角度，输出 0 ~ 2PI 之间
// ------------------------------------------------------------
float normalizeAngle(float angle);

// ------------------------------------------------------------
// 正向 Clarke + Park：Ia, Ib → Iq
// 对应 DengFOC 的 cal_Iq_Id()
// 参数：
//   ia, ib   : 两相电流
//   angle_el : 电角度
// 返回：Iq
// ------------------------------------------------------------
float calcIq(float ia, float ib, float angle_el);

#endif