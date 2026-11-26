#include "foc_pid.h"

FOC_PID::FOC_PID() {
    P_ = 0; I_ = 0; D_ = 0;
    ramp_ = 100000;
    limit_ = 12.6f;
    error_prev_ = 0;
    output_prev_ = 0;
    integral_prev_ = 0;
    timestamp_prev_ = micros();
}

void FOC_PID::setParams(float P, float I, float D, float ramp, float limit) {
    P_ = P;
    I_ = I;
    D_ = D;
    ramp_ = ramp;
    limit_ = limit;
}

// ------------------------------------------------------------
// 清零积分/输出/误差，用于故障恢复或切换模式时防止积分饱和后突跳
// ------------------------------------------------------------
void FOC_PID::reset() {
    integral_prev_ = 0;
    output_prev_   = 0;
    error_prev_    = 0;
    timestamp_prev_ = micros();
}

// ------------------------------------------------------------
// 对应 DengFOC 的 PIDController::operator()(error)
// ------------------------------------------------------------
float FOC_PID::update(float error) {
    uint32_t now = micros();
    float Ts = (now - timestamp_prev_) * 1e-6f;
    if (Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;

    // P 项
    float proportional = P_ * error;

    // I 项：Tustin 梯形积分
    float integral = integral_prev_ + I_ * Ts * 0.5f * (error + error_prev_);
    integral = constrain(integral, -limit_, limit_);

    // D 项
    float derivative = D_ * (error - error_prev_) / Ts;

    // 合成输出
    float output = proportional + integral + derivative;
    output = constrain(output, -limit_, limit_);

    // 输出变化率限幅
    if (ramp_ > 0) {
        float rate = (output - output_prev_) / Ts;
        if (rate > ramp_) output = output_prev_ + ramp_ * Ts;
        else if (rate < -ramp_) output = output_prev_ - ramp_ * Ts;
    }

    // 保存状态
    integral_prev_ = integral;
    output_prev_ = output;
    error_prev_ = error;
    timestamp_prev_ = now;

    return output;
}
