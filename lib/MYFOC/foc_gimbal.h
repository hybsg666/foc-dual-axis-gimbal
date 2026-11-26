#ifndef FOC_GIMBAL_H
#define FOC_GIMBAL_H

#include <Arduino.h>

#include "foc_pwm.h"   // MOTOR_0 / MOTOR_1 / NUM_MOTORS

// ============================================================
// 双轴云台指向控制（阶段三核心）
//
// 解决的问题：
//   以前只能分别给两台电机发角度（"yaw 走 90°，pitch 走 20°"），
//   两轴各按自己的速度跑，谁近谁先到 → 笔尖走的是一条折线，
//   不是从 A 到 B 的直线。
//
// 现在的做法（一次命令搞定）：
//   1. 输入一个**空间方向向量** (x, y, z)，不是关节角
//   2. 逆运动学算出 yaw / pitch 两个关节角
//   3. 两轴各自规划 S 曲线，但**把时长对齐到同一个 T**
//      —— 行程短的那轴自动降速，保证同起同停
//   4. 两轴一起进轨迹队列执行
//
// 数学：
//   yaw   = -atan2(x, y)          （本项目 yaw 正方向 = 顺时针，与右手系相反）
//   pitch =  atan2(z, hypot(x,y)) （pitch 正 = 抬头）
//   关节角 = 世界角 + 零位偏置（装配后标定，见 foc_gimbal_set_offset）
//
// 同步是怎么做到的（时间重标定 time scaling）：
//   只让两轴的**总时长相同**是不够的 —— S 曲线的形状会随速度变化
//   （小行程跑不到峰值加速度，走的是"纯 jerk"分支），两轴中途会一前一后。
//   做法：取两轴里**最大的行程 span** 规划一条 S 曲线，
//   两轴都以它为标尺，位置 = 起点 + 本轴行程 ×（标尺进度），
//   于是任一时刻两轴的"完成度"完全相同，同时起步、同时到位。
//   小行程轴的峰值速度/加速度自然按 dist/span 等比缩小，绝不会超限。
// ============================================================

// 电机角色（按 docs/标定记录.md：M0 = yaw 水平旋转，M1 = pitch 俯仰）
#define GIMBAL_YAW_MOTOR   MOTOR_0
#define GIMBAL_PITCH_MOTOR MOTOR_1

// 指向一个空间方向。x/y/z 不必归一化（内部只用方向）。
// vmax = 速度上限 rad/s（0 表示用默认 GIMBAL_DEFAULT_SPEED）
// 返回 false = 方向向量为零 / 参数非法
bool foc_gimbal_point(float x, float y, float z, float vmax);

// 零位偏置（度）：装配后若"指向正前方"时关节读数不是 0，用它补上
void  foc_gimbal_set_offset(float yaw_deg, float pitch_deg);
void  foc_gimbal_get_offset(float* yaw_deg, float* pitch_deg);

// yaw 关节方向符号（+1 / -1）。装配后若"发右转却往左走"，翻转它即可，
// 不用改代码重编译。默认 -1：按 docs/标定记录.md（yaw 正 = 顺时针）。
void  foc_gimbal_set_yaw_sign(float sign);
float foc_gimbal_get_yaw_sign();

// 只做逆解不执行（给串口预览 / PC 侧验证用）
void  foc_gimbal_solve(float x, float y, float z,
                       float* yaw_world_deg, float* pitch_world_deg);

#endif
