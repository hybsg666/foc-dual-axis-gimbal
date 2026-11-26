#ifndef FOC_PWM_H
#define FOC_PWM_H

#include <Arduino.h>

// 电机索引（也用于其它模块：foc_control / foc_protection / foc_config）
#define MOTOR_0 0
#define MOTOR_1 1
#define NUM_MOTORS 2

// ------------------------------------------------------------
// 初始化单个电机的三路 PWM + 使能引脚
// 参数：
//   motor   : 电机索引（0 或 1）
//   pinA/pinB/pinC : 三相 PWM 引脚
//   pinEN          : 驱动使能引脚（高电平使能）
//   freq           : PWM 频率，默认 30000Hz
//   resolution     : PWM 分辨率，默认 8 位（0~255）
// 说明：电机 0 用 LEDC 通道 0/1/2，电机 1 用 3/4/5
// ------------------------------------------------------------
void pwm_init(int motor, int pinA, int pinB, int pinC, int pinEN,
              uint32_t freq = 30000, uint8_t resolution = 8);

// 使能 / 失能指定电机
void pwm_enable(int motor);
void pwm_disable(int motor);

// 设置指定电机三相占空比（0.0 ~ 1.0）
void pwm_set_duty(int motor, float dc_a, float dc_b, float dc_c);

// ------------------------------------------------------------
// 开环力矩输出（SVPWM）
// 逆 Park + 逆 Clarke + 零序注入，把 Uq 和电角度转成三相占空比
// ------------------------------------------------------------
void pwm_set_torque_openloop(int motor, float Uq, float angle_el, float vbus);

#endif
