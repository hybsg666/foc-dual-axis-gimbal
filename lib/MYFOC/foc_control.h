#ifndef FOC_CONTROL_H
#define FOC_CONTROL_H

#include <Arduino.h>

#include "foc_motor.h"   // MODE_*、MotorConfig、FOC_Motor

// ============================================================
// 协调层
//
// 原来 foc_control.cpp 里「每台电机各一份」的逻辑已全部收进 FOC_Motor 类。
// 这里只负责：
//   1. 持有 FOC_Motor g_motors[NUM_MOTORS]
//   2. 上电初始化顺序（先全部 begin，再全部 start —— 保证电流零偏是在
//      两颗电机都静止且不使能的状态下校准的）
//   3. 每 loop 遍历所有电机调 update()
//   4. 把遥测拼成一行 14 列打印
//   5. 旧的 foc_* 自由函数 API 保留为一行转发
//      （foc_serial.cpp / foc_can.cpp 一处都不用改）
// ============================================================

// -------- 面向对象的新入口：直接拿某一台电机 --------
// 用法：foc_motor(0).setTargetPos(1.57f);
FOC_Motor&       foc_motor(int motor);

// -------- 初始化（两个电机都初始化 + 对齐 + 加载 NVS）--------
void foc_control_init();

// -------- 每 loop 调用，两个电机各跑一遍三环 + 输出 PWM --------
void foc_control_update();

// -------- 打印遥测：一行 14 列，M0 前 7 列 + M1 后 7 列 --------
void foc_control_print();

// -------- 设置目标（motor = MOTOR_0 / MOTOR_1）--------
void foc_set_mode(int motor, int mode);
void foc_set_target_iq(int motor, float iq);
void foc_set_target_vel(int motor, float vel);
void foc_set_target_pos(int motor, float pos);
// 位置平滑开关（默认开）。CANopen 的 CSP 模式（主站每 20ms 换目标）要关掉，
// 否则每段都重新规划 S 曲线，速度被反复清零，跟不上主站。
void foc_set_smooth(int motor, bool en);

// -------- 读取 --------
int   foc_get_mode(int motor);
float foc_get_target_iq(int motor);
float foc_get_target_vel(int motor);
float foc_get_target_pos(int motor);

// -------- 读取内部状态，给 CAN 反馈用 --------
float foc_control_get_full(int motor);
// 归一化 + 用户坐标（user_sign 已乘）的角度：逆解/显示用这个，
// 累积角跨圈（-280° 之类）时不会把行程算成大圈。
float foc_control_get_full_near(int motor);
float foc_control_get_velocity(int motor);
float foc_control_get_iq(int motor);

// -------- 对外坐标符号（M1 编码器装反 → -1；显示/换算用）--------
float foc_get_user_sign(int motor);

// -------- 保护（给 CANopen 的 CiA 402 用，别在业务层直接读 foc_protection）--------
// foc_get_fault 返回故障码位标志（0 = 正常），位含义见 foc_protection.h 的 FAULT_*
// 有了这两个，CANopen 才能做到"底层过流 → 上层状态字变 0x0218 + 自动发 EMCY"。
int  foc_get_fault(int motor);
void foc_clear_fault(int motor);

// 告诉保护模块"刚收到一条运动指令"。
//   ★ CANopen 必须调它：串口命令会自动调，但 PDO/SDO 走的是另一条路，
//     不调的话，主站每 20ms 喂一帧 CSP 目标，固件却以为"100 秒没人理我" → 误报指令超时。
void foc_notify_cmd(int motor);

// 控制环实际频率（Hz）。任何让单次 update 变慢的改动都会降频，
// 而降频会让同一套 PID 参数每步积分量变大 → 可能失稳，调参前先看这个数。
float foc_control_get_hz();

// -------- 手动重标定指定电机 --------
void foc_control_recalibrate(int motor);

// -------- 清空指定电机的 NVS --------
void foc_control_clear_nvs(int motor);

// -------- 切换指定电机的对齐模式（dev/prod）并存入 NVS --------
void foc_control_toggle_align_mode(int motor);
bool foc_control_get_align_mode(int motor);   // true = ALWAYS(dev)

// -------- 位置软限位（防多圈绕线 / 防撞结构，存入 NVS）--------
void  foc_set_pos_limit(int motor, float lo_rad, float hi_rad);
float foc_get_pos_limit_lo(int motor);

// 位置环 P（度为单位）。等效刚度 = k × 57.3 rad/s per rad。
void  foc_set_pos_kp(int motor, float k);
float foc_get_pos_kp(int motor);

// 位置环速度阻尼 Kd（0 = 关闭）。纯 P 无阻尼会等幅振荡，Kd 补阻尼比。
void  foc_set_pos_kd(int motor, float d);
float foc_get_pos_kd(int motor);

// ★ 装配标定：把当前位置定为云台 0°（写 NVS，顺带把限位重置成 [0,180°]）
void  foc_calibrate_zero(int motor);
// 用指定的编码器绝对角（rad）做云台零点；不改软限位（误标定恢复用）
void  foc_calibrate_zero_at(int motor, float abs_mech);
float foc_get_zero_offset(int motor);   // 零点在编码器坐标里的绝对角 rad

// ★ 速度环 P：串级系统里真正决定阻尼比的旋钮（0.02 ~ 1.0）。
//   位置环 Kd 只能间接影响它（被 Pv 小基数稀释），M0 的摆靠这一项压制。
void  foc_set_vel_kp(int motor, float k);
float foc_get_vel_kp(int motor);

// 速度环输出（iq）限幅 A = 该轴最大力矩上限
void  foc_set_vel_iq_limit(int motor, float a);
float foc_get_vel_iq_limit(int motor);
float foc_get_pos_limit_hi(int motor);
bool  foc_get_pos_limit_hit(int motor);       // 上一次设目标是否被钳位

// -------- 测量模式（全局遥测开关，量化测试专用）--------
void foc_control_set_measure(bool on);
bool foc_control_get_measure();

// -------- 诊断模式（抓偶发 bug：关遥测，只留 [TRAJ]/[ENC]/[BIGERR] 诊断行）--------
void foc_control_set_diag(bool on);
bool foc_control_get_diag();

// -------- 轨迹指令队列（双轴云台核心）--------
// 入队一条指令：pos = 目标机械角度(rad)，speed = 速度(rad/s)
// 返回 false 表示队列满，入队失败
// dwell_ms = 该段到位后原地停顿的毫秒数（0 = 不停，段间连续）
bool foc_queue_push(int motor, float pos, float speed, float dwell_ms = 0.0f);
// 入队一条"归一化同步"命令（云台指向用）：两轴给同一个 vnorm 就完全同步
//   from = 起点绝对角，dist = 有符号行程，span = 本组最大行程（共用标尺），vmax = 速度上限
bool foc_queue_push_norm(int motor, float from, float dist, float span, float vmax);
// 队列里还剩几条待执行指令
int  foc_queue_count(int motor);
// 清空队列并立即停止（回到速度模式 0）
void foc_queue_stop(int motor);
// 清空队列后开始执行（切到轨迹模式）
void foc_queue_start(int motor);

#endif
