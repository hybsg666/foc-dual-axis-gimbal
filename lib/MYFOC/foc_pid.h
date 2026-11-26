#ifndef FOC_PID_H
#define FOC_PID_H

#include <Arduino.h>

// ------------------------------------------------------------
// PID 控制器类
// 对应 DengFOC 的 PIDController
// 注意：成员变量用 xxx_ 后缀，不用 _xxx 前缀
//       因为 C++ 里 _X 这种下划线+大写字母是系统保留名
// ------------------------------------------------------------
class FOC_PID {
public:
    FOC_PID();

    void setParams(float P, float I, float D, float ramp, float limit);

    // 只改输出限幅，不动 P/I/D。用于"大误差时临时放开力矩、接近目标再收回"
    void setLimit(float limit) { limit_ = limit; }
    float getLimit() const { return limit_; }

    // 清零积分/微分/上一次输出，避免故障恢复或切换模式后 windup 导致突跳
    void reset();

    // 只清积分项（保留 P/D 状态）。换向瞬间调用，避免上一步的积分残留
    // 让反向输出被削到很小（表现为"反向转不动、正向很猛"）
    void resetIntegral() { integral_prev_ = 0.0f; }

    // 对应 DengFOC 的 PIDController::operator()(error)
    float update(float error);

private:
    float P_, I_, D_;
    float ramp_;
    float limit_;

    float error_prev_;
    float output_prev_;
    float integral_prev_;
    uint32_t timestamp_prev_;
};

#endif