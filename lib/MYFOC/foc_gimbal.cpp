#include "foc_gimbal.h"

#include <math.h>

#include "foc_control.h"
#include "foc_motor.h"
#include "foc_pwm.h"
#include "foc_trajectory.h"

namespace {

// 角度差折回 [-π, π]
inline float wrapPi(float x) { return atan2f(sinf(x), cosf(x)); }

// 零位偏置：装配后若"指向正前方"时关节读数不在地理零位，用它补（单位 rad）
float g_yaw_off   = 0.0f;
float g_pitch_off = 0.0f;
float g_yaw_sign  = -1.0f;   // 关节角 = sign × atan2(x,y) + offset

constexpr float kDefaultSpeed = 5.0f;    // rad/s
constexpr float R2D           = 57.29578f;

}  // namespace

// ============================================================
bool foc_gimbal_point(float x, float y, float z, float vmax) {
  float horiz = sqrtf(x * x + y * y);
  if (horiz < 1e-6f && fabsf(z) < 1e-6f) return false;   // 零向量，方向无意义
  if (vmax <= 0.0f) vmax = kDefaultSpeed;

  // ---------------- 1. 逆运动学 ----------------
  float yaw_w   = atan2f(x, y);          // 水平方位角 [-π, π]
  float pitch_w = atan2f(z, horiz);      // 仰角 [-π/2, π/2]

  // yaw 关节正方向 = 顺时针（与右手系相反，见 docs/标定记录.md）→ 取负
  float yaw_j   = g_yaw_sign * yaw_w + g_yaw_off;
  float pitch_j =  pitch_w + g_pitch_off;

  const int MY = GIMBAL_YAW_MOTOR;
  const int MP = GIMBAL_PITCH_MOTOR;

  // ★ 2026-11-18：用归一化 + 用户坐标的角度。
  //   原来用 foc_control_get_full()（内部坐标、未归一化）：
  //   · M0 累积角落在 -280° 时，ty=cy+dy 被 0~105° 的限位钳成 0 → 行程变 +280° → 撞限位
  //   · M1 的 user_sign=-1，内部角是负的，与 0~180° 的用户限位不是一套坐标 → 长期误钳
  float cy = foc_control_get_full_near(MY);
  float cp = foc_control_get_full_near(MP);
  float dy = wrapPi(yaw_j   - cy);       // 走最短路径，最多转半圈
  float dp = wrapPi(pitch_j - cp);

  // ---------------- 2. 软限位先钳 ----------------
  // 必须在规划前钳：否则轨迹按原距离规划，执行时再被钳，两轴时长就被破坏，
  // 同步失效（一轴提前停，笔尖又走折线）。
  float ty = cy + dy;
  float tp = cp + dp;
  bool  clamped = false;

  float lo = foc_get_pos_limit_lo(MY), hi = foc_get_pos_limit_hi(MY);
  if (ty < lo) { ty = lo; clamped = true; }
  else if (ty > hi) { ty = hi; clamped = true; }
  dy = ty - cy;

  lo = foc_get_pos_limit_lo(MP); hi = foc_get_pos_limit_hi(MP);
  if (tp < lo) { tp = lo; clamped = true; }
  else if (tp > hi) { tp = hi; clamped = true; }
  dp = tp - cp;

  // ---------------- 3. 共用标尺：本组里最大的行程 ----------------
  //   为什么不用"归一化到 1"：S 曲线的 jerk 限幅对**距离**敏感，
  //   把 90° 缩成 1.0 之后再规划，最短时间会被 jerk 卡在 0.64s，
  //   反而比真实的 0.75s 还"慢不下来"。直接用最大行程当标尺，
  //   参数仍是原来的 A/J/vmax，物理含义不变。
  float span = fmaxf(fabsf(dy), fabsf(dp));

  TrajState t;
  traj_begin(t, 0.0f, span, vmax, foc_param::TRAJ_ACCEL, foc_param::TRAJ_JERK);
  float T = traj_duration(t);
  if (T <= 0.0f) T = 0.02f;

  // ---------------- 4. 两轴同时入队启动 ----------------
  foc_queue_stop(MY);
  foc_queue_stop(MP);
  // 行程≈0 的轴不入队（保持力矩锁在原地，不参与运动）
  if (fabsf(dy) > 1e-4f) foc_queue_push_norm(MY, cy, dy, span, vmax);
  if (fabsf(dp) > 1e-4f) foc_queue_push_norm(MP, cp, dp, span, vmax);
  foc_queue_start(MY);
  foc_queue_start(MP);

  // ---------------- 5. 日志 ----------------
  //   两轴峰值速度 = dist/span × span轴的峰值速度，一般远低于 vmax（小轴更慢）
  Serial.printf("[GIMBAL] dir=(%.3f,%.3f,%.3f) -> yaw=%.1f deg pitch=%.1f deg\n",
                (double)x, (double)y, (double)z,
                (double)(yaw_w * R2D), (double)(pitch_w * R2D));
  Serial.printf("[GIMBAL] m%d yaw %+.1fdeg | m%d pitch %+.1fdeg | span=%.3f T=%.3fs SYNC\n",
                MY, (double)(dy * R2D), MP, (double)(dp * R2D),
                (double)span, (double)T);
  if (clamped) {
    Serial.printf("[GIMBAL] WARN: target hit pos_limit, actual yaw=%.1f pitch=%.1f deg\n",
                  (double)(ty * R2D), (double)(tp * R2D));
  }
  return true;
}

// ============================================================
void foc_gimbal_set_yaw_sign(float sign) {
  g_yaw_sign = (sign >= 0.0f) ? 1.0f : -1.0f;
  Serial.printf("[GIMBAL] yaw sign = %+.0f (\"g1,1,0\" should turn %s)\n",
                (double)g_yaw_sign, (g_yaw_sign > 0) ? "CLOCKWISE" : "COUNTER-CW");
}

float foc_gimbal_get_yaw_sign() { return g_yaw_sign; }

void foc_gimbal_set_offset(float yaw_deg, float pitch_deg) {
  g_yaw_off   = yaw_deg / R2D;
  g_pitch_off = pitch_deg / R2D;
  Serial.printf("[GIMBAL] zero offset set: yaw=%+.1f deg pitch=%+.1f deg\n",
                (double)yaw_deg, (double)pitch_deg);
}

void foc_gimbal_get_offset(float* yaw_deg, float* pitch_deg) {
  if (yaw_deg)   *yaw_deg   = g_yaw_off * R2D;
  if (pitch_deg) *pitch_deg = g_pitch_off * R2D;
}

// ============================================================
void foc_gimbal_solve(float x, float y, float z,
                      float* yaw_world_deg, float* pitch_world_deg) {
  float horiz = sqrtf(x * x + y * y);
  if (yaw_world_deg)   *yaw_world_deg   = atan2f(x, y) * R2D;
  if (pitch_world_deg) *pitch_world_deg = atan2f(z, horiz) * R2D;
}
