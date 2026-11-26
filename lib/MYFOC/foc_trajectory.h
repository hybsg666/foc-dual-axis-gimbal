#ifndef FOC_TRAJECTORY_H
#define FOC_TRAJECTORY_H

#include <Arduino.h>

// ============================================================
// 运动轨迹生成器（每台电机一份状态）
//
// 两条路径，由 TRAJ_SCURVE_ENABLE 选择：
//
// 【1】梯形速度（旧）：加速-匀速-减速，加速度是阶跃的。
//     问题：加速度在一瞬间从 0 跳到 30 rad/s²，jerk（加加速度）理论上是无穷大
//     → 机械冲击、齿槽噪音、云台在启停瞬间"抖一下"，齿轮/打印件受冲击载荷。
//
// 【2】S 曲线（新，默认）：7 段式，加速度也是斜坡变化的。
//     共 7 段：加加速 → 匀加速 → 减加速 → 匀速 → 加减速 → 匀减速 → 减减速
//     加速度限幅 A，加速度变化率（jerk）限幅 J。
//     → 速度曲线 C1 连续，jerk 有界，启停柔和，末端无冲击。
//
// 为什么用「绝对时间查表」而不是「每拍累加」：
//   位置只由 elapsed 决定，dt 抖动（串口打印阻塞 loop、CAN 中断）只影响
//   采样点密度，不影响轨迹正确性；累加式则会把每次 dt 的误差累积成终点偏差。
//   这也是 7 段式的标准做法，多轴插补时天然同步。
// ============================================================

struct TrajState {
  // ---- 输入参数 ----
  float start;      // 起始位置（累计角度 full，rad）
  float target;     // 目标位置（累计角度 full，rad）
  float max_speed;  // 峰值速度上限 rad/s（绝对值）
  float accel;      // 峰值加速度上限 rad/s²
  float jerk;       // 加加速度上限 rad/s³（<=0 表示退回梯形）

  // ---- 输出状态 ----
  float cur;        // 当前轨迹位置
  float cur_vel;    // 当前轨迹速度（带符号，rad/s）
  bool  active;     // 是否正在运行

  // ---- S 曲线预计算结果（traj_begin 里一次算好）----
  float _Tj;        // 单个 jerk 段时长
  float _Ta;        // 匀加速段时长
  float _Tc;        // 匀速段时长
  float _T;         // 总时长
  float _dist;      // 总位移（带符号）
  float _vpk;       // 实际峰值速度（可能低于 max_speed）
  float _apk;       // 实际峰值加速度（可能低于 accel）
  float _dir;       // 方向 +1 / -1
  float _t;         // 已运行时间
};

// 开始一段轨迹：从 start 到 target
// jerk <= 0 → 走旧的梯形算法（兼容/回退用）
void traj_begin(TrajState& t, float start, float target, float max_speed, float accel,
                float jerk = 0.0f);

// 每步推进 dt 秒，返回"当前应到达的位置"
float traj_step(TrajState& t, float dt);

// 是否已到达终点
bool traj_done(const TrajState& t);

// 轨迹当前理论速度 rad/s（带符号）—— 给遥测/验证用
float traj_vel(const TrajState& t);

// 行程概览（供串口打印：预计耗时、峰值速度）
float traj_duration(const TrajState& t);

#endif
