#include "foc_trajectory.h"
#include <math.h>

// ============================================================
// 加速段（0 → 峰值速度）的位移解析式
//   t ∈ [0, 2Tj+Ta]，分三段：
//     段1 [0, Tj]         jerk = +J   加速度 0 → A
//     段2 [Tj, Tj+Ta]     jerk = 0    加速度恒为 A
//     段3 [Tj+Ta, 2Tj+Ta] jerk = -J   加速度 A → 0
//   对称性：加速度关于中点半对称 → 平均速度 = vpk/2
// ============================================================
static float s_accel(float t, float J, float A, float Tj, float Ta) {
  if (t <= 0.0f) return 0.0f;
  float s1 = J * Tj * Tj * Tj / 6.0f;   // 段1 末端位移
  float v1 = J * Tj * Tj / 2.0f;        // 段1 末端速度

  if (t <= Tj) return J * t * t * t / 6.0f;

  if (t <= Tj + Ta) {
    float d = t - Tj;
    return s1 + v1 * d + 0.5f * A * d * d;
  }

  float s2 = s1 + v1 * Ta + 0.5f * A * Ta * Ta;
  float v2 = v1 + A * Ta;
  float d  = t - (Tj + Ta);
  if (d > Tj) d = Tj;
  return s2 + v2 * d + 0.5f * A * d * d - J * d * d * d / 6.0f;
}

// 加速段速度解析式
static float v_accel(float t, float J, float A, float Tj, float Ta) {
  if (t <= 0.0f) return 0.0f;
  if (t <= Tj) return J * t * t / 2.0f;
  float v1 = J * Tj * Tj / 2.0f;
  if (t <= Tj + Ta) return v1 + A * (t - Tj);
  float v2 = v1 + A * Ta;
  float d  = t - (Tj + Ta);
  if (d > Tj) d = Tj;
  return v2 + A * d - J * d * d / 2.0f;
}

// ============================================================
// 开始一段轨迹
// ============================================================
void traj_begin(TrajState& t, float start, float target, float max_speed, float accel,
                float jerk) {
  t.start     = start;
  t.target    = target;
  t.max_speed = fabsf(max_speed);
  t.accel     = (accel > 0.0f) ? accel : 1.0f;
  t.jerk      = jerk;
  t.cur       = start;
  t.cur_vel   = 0.0f;
  t._t        = 0.0f;
  t._dist     = target - start;
  t._dir      = (t._dist >= 0.0f) ? 1.0f : -1.0f;
  t._vpk      = 0.0f;
  t._apk      = 0.0f;
  t._Tj = t._Ta = t._Tc = t._T = 0.0f;

  float D = fabsf(t._dist);
  // 距离太小就不走了，直接判为完成（1e-4 rad ≈ 0.006°）
  if (D <= 1e-4f) {
    t.cur    = target;
    t.active = false;
    return;
  }
  t.active = true;

  // jerk <= 0 → 保持梯形算法（traj_step 里会自动走老路径）
  if (t.jerk <= 0.0f) return;

  // ---------------- S 曲线规划 ----------------
  float J = t.jerk;
  float A = t.accel;
  float V = t.max_speed;
  if (V <= 0.0f) V = A;   // 速度没设 → 用加速度兜底，避免除零

  float Tj, Ta, Tc, vpk, apk;

  // 步骤1：假定能跑到 V，先定 jerk 段与峰值加速度
  //   加速段走完要升到 V：A·(Tj+Ta) = V，且 Tj = A/J
  if (V * J <= A * A) {
    // 加速度还没升到 A 就已经到 V 了 → 实际峰值加速度低于 A
    apk = sqrtf(V * J);
    Tj  = sqrtf(V / J);
    Ta  = 0.0f;
  } else {
    apk = A;
    Tj  = A / J;
    Ta  = V / A - Tj;
    if (Ta < 0.0f) Ta = 0.0f;
  }
  vpk = V;

  // 步骤2：加速段单侧位移 = 平均速度(vpk/2) × 时长(2Tj+Ta)
  float S_acc = vpk * (Tj + 0.5f * Ta);

  if (2.0f * S_acc <= D) {
    // 距离够：中间加匀速段
    Tc = (D - 2.0f * S_acc) / vpk;
  } else {
    // 距离不够：降峰值速度，没有匀速段
    Tc = 0.0f;
    //   D = 2·(vp·Tj + vp·Ta/2) 且 Ta = vp/apk - Tj
    //   → vp²/apk + vp·Tj - D = 0
    float vp     = 0.5f * apk * (-Tj + sqrtf(Tj * Tj + 4.0f * D / apk));
    float vp_min = apk * Tj;          // Ta = 0 时的峰值速度
    if (vp >= vp_min) {
      vpk = vp;
      Ta  = vp / apk - Tj;
    } else {
      // 加速度也升不到 apk → 只有 jerk 段（Ta = Tc = 0）
      //   D = 2·J·Tj³  →  Tj = (D / 2J)^(1/3)
      Tj  = cbrtf(D / (2.0f * J));
      apk = J * Tj;
      vpk = J * Tj * Tj;
      Ta  = 0.0f;
    }
  }

  t._Tj  = Tj;
  t._Ta  = Ta;
  t._Tc  = Tc;
  t._vpk = vpk;
  t._apk = apk;
  t._T   = 2.0f * (2.0f * Tj + Ta) + Tc;
}

// ============================================================
// 梯形算法（旧版，jerk <= 0 时使用）
// ============================================================
static float traj_step_trapezoid(TrajState& t, float dt) {
  float dir       = (t.target >= t.cur) ? 1.0f : -1.0f;
  float remaining = fabsf(t.target - t.cur);
  float stop_dist = (t.cur_vel * t.cur_vel) / (2.0f * t.accel);

  if (remaining <= stop_dist) {
    float dec = t.accel * dt;
    if (fabsf(t.cur_vel) <= dec) {
      t.cur_vel = 0.0f;
    } else {
      t.cur_vel -= dir * dec;
    }
  } else if (fabsf(t.cur_vel) < t.max_speed) {
    t.cur_vel += dir * t.accel * dt;
    if (fabsf(t.cur_vel) > t.max_speed) t.cur_vel = dir * t.max_speed;
  }

  t.cur += t.cur_vel * dt;

  if (dir > 0.0f) {
    if (t.cur >= t.target) { t.cur = t.target; t.cur_vel = 0.0f; t.active = false; }
  } else {
    if (t.cur <= t.target) { t.cur = t.target; t.cur_vel = 0.0f; t.active = false; }
  }
  return t.cur;
}

// ============================================================
// 每步推进 dt 秒
// ============================================================
float traj_step(TrajState& t, float dt) {
  if (!t.active) return t.target;
  if (dt <= 0.0f) dt = 0.001f;

  // ---------- 梯形回退路径 ----------
  if (t.jerk <= 0.0f) return traj_step_trapezoid(t, dt);

  // ---------- S 曲线：按绝对时间取值，dt 抖动不影响终点 ----------
  t._t += dt;
  if (t._t > 1000.0f) t._t = 1000.0f;   // 溢出兜底

  float D     = fabsf(t._dist);
  float Tj    = t._Tj,  Ta = t._Ta, Tc = t._Tc;
  float T_acc = 2.0f * Tj + Ta;
  float s;

  if (t._t >= t._T) {
    // 到点：精确落在目标上
    s = D;
    t._t = t._T;
    t.cur_vel = 0.0f;
    t.active  = false;
  } else if (t._t < T_acc) {
    s = s_accel(t._t, t.jerk, t._apk, Tj, Ta);
  } else if (t._t < T_acc + Tc) {
    s = s_accel(T_acc, t.jerk, t._apk, Tj, Ta) + t._vpk * (t._t - T_acc);
  } else {
    // 减速段 = 加速段的镜像，保证末端位移精确等于 D
    s = D - s_accel(t._T - t._t, t.jerk, t._apk, Tj, Ta);
  }

  t.cur = t.start + t._dir * s;
  t.cur_vel = traj_vel(t);
  return t.cur;
}

// ============================================================
// 当前理论速度（带符号）
// ============================================================
float traj_vel(const TrajState& t) {
  if (!t.active || t.jerk <= 0.0f) return t.cur_vel;

  float T_acc = 2.0f * t._Tj + t._Ta;
  float v;

  if (t._t <= 0.0f) {
    v = 0.0f;
  } else if (t._t < T_acc) {
    v = v_accel(t._t, t.jerk, t._apk, t._Tj, t._Ta);
  } else if (t._t < T_acc + t._Tc) {
    v = t._vpk;
  } else if (t._t < t._T) {
    v = v_accel(t._T - t._t, t.jerk, t._apk, t._Tj, t._Ta);
  } else {
    v = 0.0f;
  }
  return t._dir * v;
}

// ============================================================
bool traj_done(const TrajState& t) {
  return !t.active;
}

float traj_duration(const TrajState& t) {
  return (t.jerk > 0.0f) ? t._T : 0.0f;
}
