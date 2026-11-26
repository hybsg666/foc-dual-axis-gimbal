// ============================================================
// FOC_Motor 实现
//
// ★ 本文件是原 foc_control.cpp 里「每台电机各一份」的那部分逻辑的逐行搬移，
//   控制行为、参数、打印内容全部保持不变 —— 这是一次纯结构重构，不是调参。
//   原有的自由函数 API 在 foc_control.cpp 里保留为薄封装（一行转发），
//   所以 foc_serial.cpp / foc_can.cpp / foc_protection.cpp 一处都不用改。
// ============================================================

#include "foc_motor.h"

#include <math.h>

#include "foc_config.h"
#include "foc_math.h"
#include "foc_protection.h"
#include "foc_pwm.h"

namespace {

// 角度差折回 [-π, π]
inline float wrapPi(float x) { return atan2f(sinf(x), cosf(x)); }

// 对齐时施加的固定电角度（3π/2）
constexpr float kAlignElectricAngle = 4.71238898038f;

// 故障码 → 人话（串口提示用；代码里故障码是位标志，可能叠加）
const char* faultName(int code) {
  switch (code) {
    case FAULT_OVERCURRENT: return "OVERCURRENT 过流";
    case FAULT_STALL:       return "STALL 堵转";
    case FAULT_POS_ERROR:   return "POS_ERROR 位置超差";
    case FAULT_CMD_TIMEOUT: return "CMD_TIMEOUT 指令超时";
    default:                return "MULTI 复合故障";
  }
}

}  // namespace

// ============================================================
// 阶段 1：硬件 + 电流零偏 + 三环 PID（不使能、不对齐）
// ============================================================
void FOC_Motor::begin(const MotorConfig& cfg) {
  _cfg = cfg;

  pwm_init(_cfg.index, _cfg.pin_a, _cfg.pin_b, _cfg.pin_c, _cfg.pin_en);
  _encoder.init(_cfg.wire, _cfg.sda, _cfg.scl);
  _current.init(_cfg.adc_a, _cfg.adc_b);

  // 三环 PID 参数
  // 关键：速度环 I=1.0（强积分，快速建立稳态力矩、天然低通抑制噪声）+ 输出限幅 0.5A
  //   （DengFOC 用 0.5A，这里之前用 IQ_LIMIT=2.0A，4 倍大会把噪声放大成 4 倍力矩纹波）。
  // ★ 速度环 P 从 0.02 提到 VEL_KP_BASE(0.15)：
  //   串级系统的阻尼比由**内环 P** 决定。P=0.02 时 M0（竖直 yaw，无重力矩）是
  //   ζ≈0.1 的无阻尼扭摆，上电/手碰就激起 1~2Hz 持续摆动；外环 Kd 只能把等效
  //   阻尼从 0.02 抬到 0.02×(1+Kd)，所以"Kd 从 0.3 加到 1.0 完全没区别"。
  //   详见 foc_motor.h 里 VEL_KP_BASE 的推导。
  _current_pid.setParams(5.0f, 200.0f, 0.0f, 100000.0f, _cfg.vbus * 0.5f);
  _vel_kp = _cfg.vel_kp;               // 按轴给：yaw 0.15 / pitch 0.05
  _velocity_pid.setParams(_vel_kp, foc_param::VEL_KI_BASE, 0.0f, 100000.0f, _vel_iq_limit);
  _pos_kp = foc_param::POS_KP_DEFAULT;
  _position_pid.setParams(_pos_kp, 0.0f, 0.0f, 100000.0f, _cfg.vel_limit);

  _mode           = MODE_VELOCITY;
  _boot_ok        = true;
  _stuck_since_ms = 0;
  _stuck_reported = false;
}

// ============================================================
// 阶段 2：读 NVS → 使能 → （按需）对齐 → 存 NVS
// 与原 initMotorAlign() 完全一致
// ============================================================
bool FOC_Motor::start() {
  const int motor = _cfg.index;

  _align_always = foc_config_load_align_mode(motor);

  // ---- 云台零点（装配标定，存 NVS）----
  //   ★ 关键：原来 _angle_offset 在 encoder.init() 里被设成"上电那一刻的角度"，
  //     于是每换一个上电姿态，p0 对应的物理位置就变一次，p 命令完全不可复现。
  //     现在：标定过就从 NVS 读固定零点，没标定过才退回"上电位置=0"。
  {
    float z = foc_config_load_pos_zero(motor);
    if (!isnan(z)) {
      _encoder.setAngleOffset(z);
      _zero_abs = z;
      Serial.printf("[main] motor %d 云台零点 = 编码器 %.4f rad (NVS)\n", motor, z);
    } else {
      _encoder.update();
      _zero_abs = _encoder.getAbsAngle();   // 老行为：上电位置当 0°
      Serial.printf("[main] motor %d 未标定云台零点 -> 以当前上电位置为 0°"
                    "（发 '%dh' 可把当前位置标定为 0°）\n", motor, motor);
    }
  }

  // 软限位：优先用 NVS 里标定过的范围，没有就用默认 [0, 180°]
  {
    float lo = foc_param::POS_LIMIT_LO_DEFAULT;
    float hi = foc_param::POS_LIMIT_HI_DEFAULT;
    bool  from_nvs = foc_config_load_pos_limit(motor, lo, hi);
    // ★ user_sign=-1 的轴（M1）：NVS 里存的可能是老固件写的"对外坐标 [0,180°]"，
    //   而内部应当是 [-180°,0]。区间整体落在正半轴 → 判定为旧值，镜像一次并存回。
    if (_cfg.user_sign < 0.0f && lo >= 0.0f && hi > 0.0f) {
      float nl = -hi;
      float nh = -lo;
      lo = nl;
      hi = nh;
      foc_config_save_pos_limit(motor, lo, hi);
      Serial.printf("[main] motor %d user_sign=-1：pos_limit 已迁移为内部 [%.1f, %.1f] deg\n",
                    motor, (double)(_cfg.user_sign * lo) * 57.2958f,
                    (double)(_cfg.user_sign * hi) * 57.2958f);
    }
    if (lo < hi) { _pos_lo = lo; _pos_hi = hi; }
    // 显示用「用户坐标」：内部 [-180,0] × sign(-1) = [180,0]，端点会颠倒，先排序再打印
    float d_lo = _cfg.user_sign * _pos_lo;
    float d_hi = _cfg.user_sign * _pos_hi;
    if (d_lo > d_hi) { float t = d_lo; d_lo = d_hi; d_hi = t; }
    Serial.printf("[main] motor %d pos_limit = [%.1f, %.1f] deg (%s)\n",
                  motor, (double)(d_lo * 57.2958f), (double)(d_hi * 57.2958f),
                  from_nvs ? "NVS" : "default");
  }

  float saved_zero = foc_config_load_zero_angle(motor);
  bool  stored_valid = (!isnan(saved_zero)) && (saved_zero >= 0.0f) && (saved_zero < 6.2831853f);

  Serial.printf("[main] motor %d ======== BOOT ========\n", motor);
  Serial.printf("[main] motor %d mode = %s\n", motor, _align_always ? "ALWAYS (dev)" : "STORED (prod)");
  if (isnan(saved_zero)) Serial.printf("[main] motor %d stored angle = (none)\n", motor);
  else                   Serial.printf("[main] motor %d stored angle = %.4f\n", motor, saved_zero);

  pwm_enable(motor);

  if (_align_always || !stored_valid) {
    Serial.printf("[main] motor %d >>>>> 将要标定：轴会动一下(吸一下)，这是正常的 <<<<<\n", motor);
    if (!align()) {
      // ★ 关键：这里必须把电机"松掉"，不能只是 return。
      //   原来只打一行日志就返回，电机仍处于使能状态，而零位还是旧值/0，
      //   下一拍 update() 用错误的电气角做 Park 变换 → 换相错乱 → 大范围乱抽。
      pwm_set_duty(motor, 0.5f, 0.5f, 0.5f);   // 三相等电位 = coast，不出力也不刹车
      _boot_ok = false;
      Serial.printf("[main] motor %d align failed -> 已 coast，这台不参与控制"
                    "（发 '%dc' 重标定，或 '%da' 切 STORED 用存好的零位）\n", motor, motor, motor);
      return false;
    }
    if (!stored_valid || fabsf(saved_zero - _zero_electric_angle) > 0.005f) {
      foc_config_save_zero_angle(motor, _zero_electric_angle);
    } else {
      Serial.printf("[config] motor %d zero_angle unchanged, skip flash write\n", motor);
    }
  } else {
    _zero_electric_angle = saved_zero;
    Serial.printf("[main] motor %d >>>>> 不标定：轴应完全静止 <<<<<\n", motor);
    Serial.printf("[main] motor %d skip align, using stored = %.4f\n", motor, _zero_electric_angle);
  }

  // ---- 上电姿态锁定（云台必需）----
  // 以前 _mode 默认 MODE_VELOCITY / target_vel=0 —— 那只是"别转"，
  // 不是"保持在这个角度"。云台带重力负载时会被慢慢拉走，
  // 用户看到的就是"上电就乱动、维持不了位置"。
  // 现在：使能完成后立刻切位置模式，目标 = 此刻的实测角 → 上电即锁在当前姿态。
  _encoder.update();
  float boot_full = _encoder.getFullAngle();

  // ★ 2026-11-18：把"越限就放宽限位"改成"把上电角归一化"。
  //   原来的做法是把 _pos_lo/_pos_hi 临时扩到包含当前累积角，副作用很隐蔽：
  //   _nearFull() 的判断依据就是"离限位端点是否在半圈内"，限位一被拉到当前角，
  //   它就认为"就在这一段弧附近"而不折算 → 位置环拿着外圈值(-280°)当坐标，
  //   用户发 0p0 时误差算成 +281° → 一路走到限位顶死（实测撞限位、iq 打满、抖动）。
  //   正确做法：限位保持用户标定的值不动，用 _nearFull 把累积角折算到这段弧上。
  //   若轴真的停在限位外，就锁在界外原地（位置环误差为 0，不会冲）；
  //   用户一发命令，_clampTarget 会把它领回区间内。
  _mode        = MODE_POSITION;
  _target_pos  = _nearFull(boot_full);
  _target_vel  = 0.0f;
  _vel_cmd     = 0.0f;
  _traj.active = false;
  _pos_smooth  = false;
  _current_pid.reset();
  _velocity_pid.reset();
  _position_pid.reset();

  Serial.printf("[main] motor %d 姿态锁定 mode=POSITION target=%.3f rad (%.1f deg)"
                " (实测累积角 %.3f rad)\n",
                motor, (double)_target_pos, (double)(_target_pos * 57.2958f),
                (double)boot_full);
  return true;
}

// ============================================================
// 对齐：开环把转子拉到固定电角度，测出零电角偏移
// ============================================================
bool FOC_Motor::align() {
  const int motor = _cfg.index;

  pwm_set_torque_openloop(motor, foc_param::ALIGN_UQ, kAlignElectricAngle, _cfg.vbus);
  delay(800);

  float mech = 0.0f;
  float final_spread = 1e9f;
  for (int attempt = 0; attempt < foc_param::ALIGN_RETRY; attempt++) {
    float ref = 0.0f, sum = 0.0f, mn = 1e9f, mx = -1e9f;
    for (int i = 0; i < foc_param::ALIGN_SAMPLES; i++) {
      _encoder.update();
      float ang = _encoder.getMechanicalAngle();
      if (i == 0) ref = ang;
      float d = wrapPi(ang - ref);
      sum += d;
      if (d < mn) mn = d;
      if (d > mx) mx = d;
      delay(5);
    }
    mech = normalizeAngle(ref + sum / (float)foc_param::ALIGN_SAMPLES);
    final_spread = mx - mn;
    if (final_spread < foc_param::ALIGN_SETTLE) break;
    Serial.printf("[main] motor %d align: rotor not settled (spread=%.3f rad), retry %d/%d\n",
                  motor, (double)final_spread, attempt + 1, foc_param::ALIGN_RETRY);
    delay(300);
  }

  if (!_encoder.isHealthy()) {
    pwm_set_torque_openloop(motor, 0, kAlignElectricAngle, _cfg.vbus);
    Serial.printf("[main] motor %d align ABORT: encoder not responding\n", motor);
    return false;
  }

  // 转子一直在滑（典型原因：装好云台后自重/重力把转子从对齐位置拖走）。
  // 这时采到的 mech 不是"吸住时的位置"，算出的零位必然是错的。
  // 用错的零位跑 FOC = 换相错乱 = 上电大范围抽动，所以宁可拒绝也不能凑合。
  if (final_spread > 0.10f) {
    pwm_set_torque_openloop(motor, 0, kAlignElectricAngle, _cfg.vbus);
    Serial.printf("[main] motor %d align REJECT: spread=%.3f rad (%.1f deg) 太大，"
                  "转子被外力拖走了\n", motor, (double)final_spread, (double)(final_spread * 57.2958f));
    Serial.printf("[main] motor %d !!! 建议：1) 先用手/支架把云台托住保持不动再上电；"
                  "2) 或发 '%da' 切 STORED 模式，直接用 NVS 里存好的零位\n", motor, motor);
    return false;
  }

  _zero_electric_angle = normalizeAngle((float)(_cfg.dir * _cfg.pole_pairs) * mech);
  pwm_set_torque_openloop(motor, 0, kAlignElectricAngle, _cfg.vbus);
  Serial.printf("[main] motor %d zero_electric_angle = %.4f (spread=%.3f rad)\n",
                motor, _zero_electric_angle, (double)final_spread);
  return true;
}

// ============================================================
// 手动重标定
// ============================================================
void FOC_Motor::recalibrate() {
  const int motor = _cfg.index;
  Serial.printf("[main] motor %d re-calibrating...\n", motor);

  _mode = MODE_VELOCITY;
  _target_iq = 0; _target_vel = 0; _target_pos = 0;
  _current_pid.reset();
  _velocity_pid.reset();
  _position_pid.reset();

  pwm_enable(motor);
  pwm_set_torque_openloop(motor, 0, kAlignElectricAngle, _cfg.vbus);
  delay(500);

  if (align()) {
    foc_config_save_zero_angle(motor, _zero_electric_angle);
    Serial.printf("[main] motor %d re-calibrate done\n", motor);
  } else {
    Serial.printf("[main] motor %d re-calibrate FAILED\n", motor);
  }
}

// ============================================================
// 低通滤波（状态在成员里，每台电机独立）
// ============================================================
float FOC_Motor::_iqLowPass(float x) {
  uint32_t now = micros();
  float dt = (now - _iq_filt_ts) * 1e-6f;
  if (dt < 0) dt = 1e-3f;
  else if (dt > 0.3f) {
    _iq_filt_y  = x;
    _iq_filt_ts = now;
    return x;
  }
  float alpha = 0.05f / (0.05f + dt);   // Tf = 0.05s
  float y = alpha * _iq_filt_y + (1.0f - alpha) * x;
  _iq_filt_y  = y;
  _iq_filt_ts = now;
  return y;
}

float FOC_Motor::_velLowPass(float x) {
  uint32_t now = micros();
  float dt = (now - _vel_filt_ts) * 1e-6f;
  if (dt < 0) dt = 1e-3f;
  else if (dt > 0.3f) {
    _vel_filt_y  = x;
    _vel_filt_ts = now;
    return x;
  }
  float alpha = foc_param::VEL_FILT_TF / (foc_param::VEL_FILT_TF + dt);   // 一阶低通
  float y = alpha * _vel_filt_y + (1.0f - alpha) * x;
  _vel_filt_y  = y;
  _vel_filt_ts = now;
  return y;
}

// ============================================================
// 位置环输出速度指令
// ★ 对齐 DengFOCtext10（用户亲测位置环稳定不震）的纯三环做法：
//   位置环就是纯 P=1，误差(度)直接线性映射成速度指令，【不加死区、不加低通】。
//   死区会在边界造成 vel_cmd 从 0 突然跳到 P*err 的不连续，配合编码器噪声让电机
//   在边界来回穿越 → 停止微震；位置误差低通会引入滞后、且会掩盖编码器真实噪声。
//   DengFOC 证明：这个硬件用最朴素的纯 P 就够稳，噪声由下游速度环的强积分(I=1)消化。
// ============================================================
float FOC_Motor::_positionVelCmd(float pos_err_rad) {
  float err_deg = pos_err_rad * 180.0f / PI;
  // 死区：见 foc_param::POS_DEADBAND_DEG 的说明。
  //   只砍掉"编码器量化 + 机械间隙"级别的抖动，不影响真正的大误差跟踪。
  if (fabsf(err_deg) < foc_param::POS_DEADBAND_DEG) {
    _position_pid.reset();      // 顺手清积分（当前是纯 P，留着以后加 I 也不会windup）
    return 0.0f;
  }
  return _position_pid.update(err_deg);   // 纯 P=1，单位：度
}

// ============================================================
// 位置模式分支
// ★ 回到"纯三环"：不做 180° 边界滞回。
//   上一版加过"用上一拍转向锁住误差符号"的滞回，本意是防 180° 抖方向，
//   但它依赖 _prev_vel_cmd 的符号；一旦符号和真实误差方向不一致，
//   误差就被强行翻到反方向 → 位置环绕远路 → 电机撞限位停住 → 报 4。
//   这里的净收益为负，撤掉。180° 抖方向用命令层的趋势判断（轨迹模式）解决，
//   位置模式就老老实实按 wrapPi 走最短路径。
// ============================================================
float FOC_Motor::_runPositionMode(float dt, float full) {
  // 平滑模式：位置环追 S 曲线的当前点，而不是直接追终点
  float track = _target_pos;
  if (_pos_smooth) {
    if (_traj.active) {
      traj_step(_traj, dt);
      track = _traj.cur;
    } else {
      // 轨迹跑完 → 交回终点（wrapPi 后与轨迹终点等价，不会跳）
      _pos_smooth = false;
    }
  }

  // ★ 线性误差（不再用 wrapPi 取最短路径）
  //   云台行程 0~180° 正好半圈，两个端点相差 ±π。用 wrapPi 时，
  //   噪声会让"该往正转还是往负转"在 ±π 边界随机翻转，
  //   表现就是"有时候从另一边绕过去"。线性差的方向由符号唯一确定，
  //   再配合软限位，电机永远在 [0,180°] 这段弧里来回，不会绕整圈。
  float pos_err = track - _nearFull(full);
  _vel_cmd = _positionVelCmd(pos_err);
  return pos_err;
}

// ============================================================
// 轨迹模式分支
// ============================================================
float FOC_Motor::_runTrajectoryMode(float dt, float mech, float full) {
  const int motor = _cfg.index;

  // ★ 段间停顿（dwell）：上一段已经跑完、但要在这里"停住"一段时间的场合。
  //   典型场景 = 模拟激光笔跟踪：跳到目标 → 稳住 2 秒 → 再跳下一个。
  //   停顿期间不推进轨迹，位置环继续锁 _target_pos（= 上一段终点）。
  //   millis 回绕用有符号差值比较，不会在 49 天后卡住。
  if (!_traj.active && _q_count > 0 && _dwell_until != 0) {
    if ((int32_t)(millis() - _dwell_until) < 0) {
      float perr_wait = _target_pos - _nearFull(full);
      _vel_cmd = _positionVelCmd(perr_wait);
      return perr_wait;
    }
    _dwell_until = 0;   // 停顿结束 → 允许出队下一条
  }

  // 队列非空且当前无活动轨迹 → 出队下一条
  if (!_traj.active && _q_count > 0) {
    MotionCmd cmd = _queue[_q_head];
    _q_head = (_q_head + 1) % foc_param::QUEUE_SIZE;
    _q_count--;
    _cur_dwell_ms = cmd.dwell;   // 本段到位后要停顿多久

    if (cmd.norm) {
      // ---- 归一化同步命令：规划 0→1，两轴共用同一条曲线 ----
      _sync_norm = true;
      _sync_from = cmd.from;
      _sync_dist = cmd.dist;
      _sync_span = cmd.span;
      // 用"本组最大行程 span"作为共用标尺，参数仍是全局 A/J/vmax →
      //   两轴形状完全相同，且小行程轴的速度/加速度/加加速度都按比例缩小，天然不超限
      traj_begin(_traj, 0.0f, _sync_span, cmd.speed,
                 foc_param::TRAJ_ACCEL, foc_param::TRAJ_JERK);
      Serial.printf("[TRAJ] m%d #%d/%d SYNC dist=%.3f span=%.3f T=%.3fs\n",
                    motor, _q_total - _q_count, _q_total,
                    (double)cmd.dist, (double)_sync_span, (double)traj_duration(_traj));
      foc_protection_notify_cmd(motor);
    } else {
      _sync_norm = false;

      // ★ 线性差（云台坐标）。不再用 wrapPi：
      //   0~180° 行程的两个端点相差正好 π，wrapPi 在边界上会因为编码器噪声
      //   随机选方向（正转/反转都可能），这就是"从另一边转过去"的来源。
      //   线性差的方向由符号唯一确定 → 永远在区间这段弧里走。
      float delta = cmd.pos - _nearFull(full);
      if (fabsf(delta) > PI + 0.5f) delta = wrapPi(delta);   // 累积角漂整圈的兜底

      _last_cmd_pos = cmd.pos;
      _has_cmd = true;

      // ★ 起点必须和位置环用同一个坐标（_nearFull 折算后）。
      //   直接用原始 full 的后果：上电累积角落在 -280° 而物理在 79.6° 时，
      //   轨迹会按 -280°→0° 规划出一段 +281° 的行程，电机一路走到限位顶死。
      float from_full = _nearFull(full);
      float target_full = _clampTarget(from_full + delta);
      traj_begin(_traj, from_full, target_full, cmd.speed,
                 foc_param::TRAJ_ACCEL, foc_param::TRAJ_JERK);
      // 一行打印（原来两行，每段阻塞 loop 约 12ms，长队列时会让动作变顿）：
      //   #3/20 = 第 3 段 / 共 20 段，便于一眼看出队列跑到哪儿了
      Serial.printf("[TRAJ] m%d #%d/%d dist=%.3f vpk=%.2f T=%.3fs\n",
                    motor, _q_total - _q_count, _q_total,
                    (double)fabsf(target_full - full), (double)_traj._vpk,
                    (double)traj_duration(_traj));
      foc_protection_notify_cmd(motor);  // 刷新指令时间戳，防"指令超时"误报
    }
  }

  {
    bool was_active = _traj.active;
    if (_traj.active) {
      // 推进轨迹，把"移动中的目标位置"喂给位置环
      float p = traj_step(_traj, dt);
      // norm 命令里 p 是"走完标尺的多少 rad"，进度 = p/span，再按 dist 缩放成绝对角
      _target_pos = _sync_norm ? (_sync_from + _sync_dist * (p / _sync_span)) : p;
    }
    // 段刚跑完 → 本段若要求停顿，从现在开始计时
    if (was_active && !_traj.active && _cur_dwell_ms > 0.0f) {
      _dwell_until = millis() + (uint32_t)_cur_dwell_ms;
    }
  }

  // 队列全部执行完：明确打一行，避免"以为卡住/以为还在动"
  if (!_traj.active && _q_count == 0 && _q_total > 0 && !_q_done_reported) {
    _q_done_reported = true;
    Serial.printf("[QUEUE] m%d %d segs done, hold at %.3f rad\n",
                  motor, _q_total, (double)_target_pos);
  }

  // 无论是否 active，都用位置环锁 _target_pos（轨迹完成 = 锁住最后位置）
  float perr = _target_pos - _nearFull(full);
  if (fabsf(perr) > 2.5f) {
    Serial.printf("[BIGERR] m%d target=%.3f full=%.3f err=%.3f\n",
                  motor, _target_pos, full, perr);
  }
  _vel_cmd = _positionVelCmd(perr);
  return perr;
}

// ============================================================
// 进故障瞬间：把最近 2 秒的环形快照一次性打出来（只打一次）
// ============================================================
void FOC_Motor::_printSnapOnFaultEntry(int fault) {
  Serial.printf("[SNAP] m%d fault=0x%02X, last 2s (old->new, 1 row/100ms)\n",
                _cfg.index, fault);
  Serial.println("[SNAP]    pos    vel     iq   vcmd    err");
  for (int k = 0; k < foc_param::SNAP_FRAMES; k++) {
    const SnapFrame& s = _snap[(_snap_i + k) % foc_param::SNAP_FRAMES];
    Serial.printf("[SNAP] %6.3f %6.3f %6.3f %6.1f %6.3f\n",
                  (double)s.pos, (double)s.vel, (double)s.iq,
                  (double)s.vcmd, (double)s.err);
  }
}

// ============================================================
// 卡住早期诊断：大误差 + 几乎不动，持续 400ms 就立刻打一行，
// 不用等保护 3 秒后报故障。这一行能直接区分：
//   iq ≈ vlim       → 电流顶满，力矩不够（齿槽/摩擦/线缆阻力）
//   iq ≈ 0          → 控制没给力（编码器读数冻结 / 逻辑问题）
//   vel 不为 0 却在报 → 速度反馈有偏置
// ============================================================
void FOC_Motor::_updateStuckDiag(float pos_err, float velocity, float iq, float iq_cmd) {
  bool pos_mode = (_mode == MODE_POSITION || _mode == MODE_TRAJECTORY);
  if (pos_mode && fabsf(pos_err) > 0.35f && fabsf(velocity) < 0.3f) {
    if (_stuck_since_ms == 0) _stuck_since_ms = millis();
    if (!_stuck_reported && (millis() - _stuck_since_ms) > 400) {
      _stuck_reported = true;
      Serial.printf("[STUCK] m%d err=%.3f pos=%.3f vel=%.4f iq=%.3f iqcmd=%.3f "
                    "vcmd=%.2f vlim=%.2f mode=%d\n",
                    _cfg.index, (double)pos_err, (double)_g_full, (double)velocity,
                    (double)iq, (double)iq_cmd, (double)_vel_cmd,
                    (double)_velocity_pid.getLimit(), _mode);
    }
  } else {
    _stuck_since_ms = 0;
    _stuck_reported = false;
  }
}

// ============================================================
// 故障现场快照：每 100ms 记一帧，环形覆盖
// ============================================================
void FOC_Motor::_pushSnap(float pos, float vel, float iq, float vcmd, float err) {
  if (millis() - _snap_ts < (uint32_t)foc_param::SNAP_MS) return;
  _snap_ts = millis();
  SnapFrame& s = _snap[_snap_i];
  s.pos  = pos;
  s.vel  = vel;
  s.iq   = iq;
  s.vcmd = vcmd;
  s.err  = err;
  _snap_i = (_snap_i + 1) % foc_param::SNAP_FRAMES;
}

// ============================================================
// 保护检查
// ============================================================
void FOC_Motor::_updateProtection(float iq, float velocity, float pos_err, float full) {
  float pos_err_for_check = (_mode == MODE_POSITION || _mode == MODE_TRAJECTORY)
                              ? pos_err : 0.0f;
  float cmd_vel_for_check;
  if (_mode == MODE_VELOCITY)      cmd_vel_for_check = _target_vel;
  else if (_mode == MODE_POSITION) cmd_vel_for_check = _vel_cmd;
  else                             cmd_vel_for_check = 0.0f;

  foc_protection_update(_cfg.index, iq, velocity, pos_err_for_check,
                        cmd_vel_for_check, full, _mode);
}

// ============================================================
// 每 loop：一台电机的三环（电流/速度/位置）+ 保护 + 诊断
// 与原 runControl() 逐行等价
// ============================================================
void FOC_Motor::update() {
  const int motor = _cfg.index;

  // 上电自检没过（对齐被拒 / 编码器坏）：这台不参与控制，保持 coast 不动。
  //   绝不能用错误零位硬跑 —— 那是"大范围乱抽"的直接来源。
  if (!_boot_ok) return;

  // ---- 故障时不做任何控制 ----
  // ★ 故障中必须"喊出来"：以前这里静默 return，电机一动不动、串口也没提示，
  //   表现就是"发什么命令都不执行，像死机"。现在进入故障时打现场快照，
  //   并且每 3 秒重复一行提醒，直到有人发 'r' 清除。
  bool fault = foc_protection_is_fault(motor);
  if (fault) {
    int code = foc_protection_get_fault(motor);
    if (!_prev_fault) {
      _printSnapOnFaultEntry(code);        // 最近 2 秒 20 帧现场，只打一次
      _fault_log_ts = millis();
      Serial.printf("[FAULT] m%d code=0x%02X (%s) -> MOTOR STOPPED, "
                    "all commands ignored. Send '%dr' to clear.\n",
                    motor, code, faultName(code), motor);
    } else if (millis() - _fault_log_ts > 3000) {
      _fault_log_ts = millis();
      Serial.printf("[FAULT] m%d still faulted (code=0x%02X %s), send '%dr'\n",
                    motor, code, faultName(code), motor);
    }
    _g_iq       = 0;
    _g_velocity = 0;
    _prev_fault = true;
    return;
  }
  // 刚从故障恢复：清 PID 积分，避免 windup 猛冲
  if (_prev_fault) {
    _current_pid.reset();
    _velocity_pid.reset();
    _position_pid.reset();
    _prev_fault = false;
  }

  // 0. 控制环实际周期（串口打印会阻塞 loop，周期不稳定，用真实时间算 dt 给轨迹用）
  uint32_t now_us = micros();
  float dt = (now_us - _loop_us) * 1e-6f;
  if (dt <= 0.0f || dt > 0.05f) dt = 0.001f;  // 异常兜底 1ms
  _loop_us = now_us;

  // 1. 传感器
  _encoder.update();
  _current.update();

  float mech     = _encoder.getMechanicalAngle();
  float full     = _encoder.getFullAngle();
  float angle_el = normalizeAngle((float)(_cfg.dir * _cfg.pole_pairs) * mech
                                  - _zero_electric_angle);

  // 2. 速度反馈
  float vel_raw  = _encoder.getVelocity();
  float velocity = _velLowPass(vel_raw);

  // 3. 位置环 / 轨迹
  float pos_err_now = 0.0f;   // 本拍位置误差（位置/轨迹模式有效）
  if (_mode == MODE_POSITION) {
    pos_err_now = _runPositionMode(dt, full);
  } else if (_mode == MODE_TRAJECTORY) {
    pos_err_now = _runTrajectoryMode(dt, mech, full);
  }

  // 3.5 位置/轨迹模式的电子阻尼（辅助项）
  //   位置环给出的只是"想去多快"，纯 P 没有任何东西阻止它冲过头。
  //   这里按实测速度反向扣减 → 速度越快、指令被削得越多 → 天然刹车。
  //   ★ 注意：这一项最终只会把速度环的等效阻尼系数从 Pv 抬到 Pv·(1+Kd)，
  //     所以它**不能**单独压住大惯量轴的振荡（Pv 原来是 0.02，Kd=1.0 也只到 0.04）。
  //     真正起作用的是速度环 P（VEL_KP_BASE / 'w' 命令），这里保留作为"死区内的电子刹车"：
  //     误差进死区后 vel_cmd=0，Kd 让 vel_cmd = −Kd·ω，残余摆动被快速收住。
  if ((_mode == MODE_POSITION || _mode == MODE_TRAJECTORY) && _pos_kd > 0.0f) {
    _vel_cmd -= _pos_kd * velocity;
    if (_vel_cmd >   foc_param::VEL_LIMIT) _vel_cmd =  foc_param::VEL_LIMIT;
    if (_vel_cmd < -foc_param::VEL_LIMIT) _vel_cmd = -foc_param::VEL_LIMIT;
  }

  // 4. 速度环（纯三环串级：位置环 vel_cmd → 速度环 → 电流环，对齐 DengFOC）
  //   不再有"到位锁定/冻结积分/慢自适应"——DengFOC 证明纯三环 + 强积分(I=1)就够稳，
  //   那些冻结/切换会制造力矩突变、力矩不平衡，反而是震动来源。
  // ★ 同时也撤掉了两处"救火"逻辑：
  //   ① 换向清积分：条件 |vel_cmd| > 0.5 太松，电机在目标附近 vel_cmd 在 0 附近
  //      小幅正负抖动时会被反复误触发 → 速度环积分每几拍就被清一次 → 力矩建不起来
  //      → 电机在原地"没力气" → 误差越拖越大 → 报 4。
  //   ② 每拍 setLimit：init 里已经设好 0.5A，每拍再写一次纯属多余，
  //      反而让"限幅值"变成第二处可变量，排查时多一个干扰源。
  float iq_target_cmd;
  if (_mode == MODE_POSITION || _mode == MODE_TRAJECTORY) {
    iq_target_cmd = _velocity_pid.update(_vel_cmd - velocity);
  } else if (_mode == MODE_VELOCITY) {
    iq_target_cmd = _velocity_pid.update(_target_vel - velocity);
  } else {
    iq_target_cmd = _target_iq;
  }

  // 5. 电流环
  float iq_raw = calcIq(_current.getCurrentA(), _current.getCurrentB(), angle_el);
  float iq     = _iqLowPass(iq_raw);
  float uq     = _current_pid.update(iq_target_cmd - iq);

  // 6. 输出
  pwm_set_torque_openloop(motor, uq, angle_el, _cfg.vbus);

  // 7. 缓存给遥测用
  _g_full     = full;
  _g_velocity = velocity;
  _g_iq       = iq;
  _g_iq_cmd   = iq_target_cmd;
  _g_vel_raw  = vel_raw;
  _g_iq_raw   = iq_raw;

  // 8. 卡住早期诊断
  _updateStuckDiag(pos_err_now, velocity, iq, iq_target_cmd);

  // 8.5 故障现场快照
  _pushSnap(full, velocity, iq, _vel_cmd, pos_err_now);

  // 9. 保护检查
  _updateProtection(iq, velocity, pos_err_now, full);
}

// ============================================================
// 模式 / 目标
// ============================================================
void FOC_Motor::setMode(int m) {
  if (m == _mode) return;
  _mode = m;
  _traj.active = false;   // 切模式时丢掉上一模式的轨迹，避免位置环追到旧点
  _pos_smooth  = false;
  // 切到别的模式 → 轨迹队列作废（否则残留的 q 指令会在下次进轨迹模式时突然执行）
  _q_count  = 0;
  _q_total  = 0;
  _sync_norm = false;
  _current_pid.reset();
  _velocity_pid.reset();
  _position_pid.reset();
}

void FOC_Motor::setTargetIq(float a) {
  _target_iq = constrain(a, -_cfg.iq_limit, _cfg.iq_limit);
}

void FOC_Motor::setTargetVel(float rad_s) {
  // 对外坐标 → 内部坐标（user_sign；三环内部始终用内部坐标）
  rad_s = _cfg.user_sign * rad_s;
  _target_vel = constrain(rad_s, -foc_param::VEL_TARGET_MAX, foc_param::VEL_TARGET_MAX);
}

// ============================================================
// 位置模式的"隐式 S 曲线"
//
// 老行为（POS_SMOOTH_ENABLE=0）：p 命令把 _target_pos 一把设成终点，
//   位置环立刻看到几十度的误差，P=1 输出 vel_cmd = 误差度数，
//   直接顶到 VEL_LIMIT(30 rad/s) → 电机猛冲 → 到点前又急刹 → 过冲、抖一下。
//   这不是 PID 调不好，是"目标阶跃"本身逼出来的。
//
// 新行为：收到 p 命令先规划一条 S 曲线（从当前位置到目标），
//   位置环追的是"移动中的中间点"，误差始终只有几度 → vel_cmd 平滑上升。
//   用户指令一个字没变，还是 p1.57。
//
// 注意：_target_pos 仍然保存"用户要的终点"（遥测和外部读到的语义不变），
//       轨迹点存在 _traj.cur 里，只给位置环用。
// ============================================================
void FOC_Motor::setTargetPos(float rad) {
  // ★ 对外坐标 → 内部坐标：唯一一次符号翻转，就在这里。
  //   之后 _clampTarget / 轨迹 / 位置环 / 限位全部只用内部坐标。
  rad = _cfg.user_sign * rad;
  float tgt = _clampTarget(rad);

  if (!foc_param::POS_SMOOTH_ENABLE || !_smooth_en) {
    _traj.active = false;
    _pos_smooth  = false;
    _target_pos  = tgt;
    return;
  }

  // 起点用上一拍的实测位置（不用 _encoder.update()：
  // 多读一次会让编码器的测速时间窗变短，速度噪声变大，得不偿失。
  // 一拍 ≈ 3.7ms，5rad/s 时起点误差约 1°，位置环会在第一拍就吃掉）
  // ★ 用 _nearFull 折算：轨迹起点与位置环用的是同一个坐标，不会打架。
  float full = _nearFull(_g_full);

  // ★ 线性差，不再"走最短路径"。
  //   云台只在 0~180° 这一段弧里运动，最短路径没有任何意义，反而会在
  //   两个端点（相差正好 180°）随机选方向 → 看起来"从另一边绕过去"。
  //   这里方向完全由 target 与当前位置的差决定，配合软限位 → 永不越界。
  float end = _clampTarget(tgt);

  // 已经在目标上（<0.001 rad ≈ 0.06°）：不用规划，直接锁死
  if (fabsf(end - full) < 1e-3f) {
    _traj.active = false;
    _pos_smooth  = false;
    _target_pos  = end;
    return;
  }

  traj_begin(_traj, full, end, _pos_speed, foc_param::TRAJ_ACCEL, foc_param::TRAJ_JERK);
  _pos_smooth = _traj.active;
  _target_pos = end;          // 对外显示的终点（不是轨迹中间点）

  // 只报大于 1° 的跨步：画圆时每 20ms 来一个小步（约 0.2°），
  // 逐条打印会把 115200 的串口刷爆、还会拖慢主循环。
  // ★ 再加一道 300ms 限速（2026-11-24）：主站每 20ms 喂一个目标时，
  //   轨迹每 20ms 就重启一次，"本步要走的距离"永远等于剩余误差（几十度），
  //   1° 的阈值形同虚设 → 还是 50Hz 刷屏 → 串口阻塞 → 控制环掉频 → 电机变软。
  static uint32_t s_sm_print_ms[2] = { 0, 0 };
  const int mi = _cfg.index & 1;
  const uint32_t nowms = millis();
  if (fabsf(end - full) > 0.0175f && (uint32_t)(nowms - s_sm_print_ms[mi]) >= 300) {
    s_sm_print_ms[mi] = nowms;
    Serial.printf("[SMOOTH] m%d %.3f->%.3f rad (%+.1f deg) v=%.1f T=%.3fs\n",
                  _cfg.index, (double)full, (double)end, (double)(end - full) * 57.2958f,
                  (double)_pos_speed, (double)traj_duration(_traj));
  }
}

// ============================================================
// 软限位：钳位 + 警告（500ms 限流，避免连续指令把串口刷爆）
// 返回值 = 实际生效的目标
// ============================================================
float FOC_Motor::_clampTarget(float v) {
  if (v >= _pos_lo && v <= _pos_hi) {
    _limit_hit = false;
    return v;
  }
  float c = (v < _pos_lo) ? _pos_lo : _pos_hi;
  _limit_hit = true;
  uint32_t now = millis();
  if (now - _limit_warn_ts > 500) {
    _limit_warn_ts = now;
    Serial.printf("[LIMIT] m%d target %.1f deg clamped to %.1f deg (range %.1f ~ %.1f)\n",
                  _cfg.index, (double)(_cfg.user_sign * v) * 57.2958f,
                  (double)(_cfg.user_sign * c) * 57.2958f,
                  (double)(_cfg.user_sign * _pos_lo) * 57.2958f,
                  (double)(_cfg.user_sign * _pos_hi) * 57.2958f);
  }
  return c;
}

void FOC_Motor::setPosKp(float k) {
  if (k <= 0.0f) return;
  _pos_kp = k;
  _position_pid.setParams(_pos_kp, 0.0f, 0.0f, 100000.0f, _cfg.vel_limit);
  _position_pid.reset();
  Serial.printf("[PID] m%d pos_kp = %.3f -> 等效刚度 %.1f rad/s per rad\n",
                _cfg.index, (double)k, (double)(k * 57.2958f));
}

void FOC_Motor::setPosKd(float d) {
  if (d < 0.0f) d = 0.0f;
  if (d > 3.0f) d = 3.0f;          // 再大就等于"粘在地上"，动作会明显变慢
  _pos_kd = d;
  Serial.printf("[PID] m%d pos_kd = %.3f (速度阻尼, 0=关闭)\n",
                _cfg.index, (double)d);
}

// ============================================================
// 速度环 P —— 串级系统里真正的"阻尼旋钮"
//
// 为什么调这个而不是调位置环 Kd：
//   位置环 Kd 是 `vel_cmd -= Kd·ω`，它最终只把速度环的等效阻尼系数
//   从 Pv 抬到 Pv·(1+Kd)。Pv=0.02 时，Kd 从 0.3→1.0 只让系数 0.026→0.04，
//   阻尼比几乎不动 —— 这就是"四个命令发下去毫无区别"的数学原因。
//   直接抬 Pv 才是 1:1 地增加阻尼比 ζ = Kt·Pv/(2·J·ωn)。
//
// 上限 1.0：再高会把 12bit 编码器的速度噪声（约 0.2 rad/s）放大成
//   0.2A 级别的电流纹波，听起来是细密"嗡嗡"，而且发热。
// ============================================================
void FOC_Motor::setVelKp(float k) {
  if (k < 0.02f) k = 0.02f;
  if (k > 1.0f)  k = 1.0f;
  _vel_kp = k;
  // 只改参数，不清积分（清积分会让力矩突跳，反而激起一次摆动）
  _velocity_pid.setParams(_vel_kp, foc_param::VEL_KI_BASE, 0.0f, 100000.0f, _vel_iq_limit);
  Serial.printf("[PID] m%d vel_kp = %.3f (速度环 P, 越大越不摆、越大越吵)\n",
                _cfg.index, (double)k);
}

void FOC_Motor::setVelIqLimit(float a) {
  if (a <= 0.0f) return;
  if (a > 2.5f) a = 2.5f;              // 硬上限：再大容易把 12V 电源拉跌
  _vel_iq_limit = a;
  _velocity_pid.setLimit(a);           // 只改限幅，不清积分（避免力矩突跳）
  Serial.printf("[PID] m%d vel iq_limit = %.2f A (力矩上限)\n",
                _cfg.index, (double)a);
}

void FOC_Motor::setPosLimit(float lo_rad, float hi_rad) {
  if (lo_rad > hi_rad) {
    float t = lo_rad;
    lo_rad = hi_rad;
    hi_rad = t;
  }

  // ★ 入参是**对外坐标**（度→rad 后），这里换算成内部坐标。
  //   user_sign = -1 时区间整体镜像并交换端点：用户 [0,180°] → 内部 [-180°,0]。
  if (_cfg.user_sign < 0.0f) {
    float lo_in = -hi_rad;
    float hi_in = -lo_rad;
    lo_rad = lo_in;
    hi_rad = hi_in;
  }

  _pos_lo = lo_rad;
  _pos_hi = hi_rad;

  // 当前目标若已越界，拉回范围内，避免"改完限位但电机还冲着旧目标去"
  _target_pos = _clampTarget(_target_pos);

  float d_lo = _cfg.user_sign * lo_rad;   // 同上：换回用户坐标后端点可能颠倒
  float d_hi = _cfg.user_sign * hi_rad;
  if (d_lo > d_hi) { float t = d_lo; d_lo = d_hi; d_hi = t; }
  Serial.printf("[LIMIT] m%d range set to [%.1f, %.1f] deg (%.3f, %.3f rad)\n",
                _cfg.index, (double)(d_lo * 57.2958f), (double)(d_hi * 57.2958f),
                (double)d_lo, (double)d_hi);
  foc_config_save_pos_limit(_cfg.index, lo_rad, hi_rad);
}

// ============================================================
// 把累积角折算到"离限位区间最近的等价角"（±2π 整数倍）
//
// 为什么需要：位置误差现在用线性差（不走最短路径），这对 0~180° 的小行程是
//   唯一正确的做法 —— 但也意味着"如果 full 因为编码器漏计圈数漂了 360°，
//   误差会变成 300° 量级，电机会被逼着真的绕一整圈"。
//   这里把 full 折回最近的等价角：既保住线性差的方向正确性，又不会绕圈。
//   正常情况（full 在 [lo,hi] 内）k=0，直接原样返回，零开销。
// ============================================================
float FOC_Motor::_nearFull(float full) {
  // 行程接近整圈时不折算（这种配置下"最短路径"才是对的）
  if ((_pos_hi - _pos_lo) > 6.0f) return full;

  // ★ 只要离任一限位端点在半圈以内，就认为"物理上就在这段弧附近"，原样返回。
  //   这一条很重要：如果云台被外力推到区间外一点（比如 -20°），
  //   按"离区间中点最近的等价角"硬折算会把它折到 +340° 去，
  //   位置环就会反向绕 340° —— 正是要避免的"从另一边转过去"。
  //   只有离两个端点都超过半圈（= 编码器漏计了一整圈）才折算，
  //   那时折算到的是同一个物理位置，方向不会错。
  if (fabsf(full - _pos_lo) < PI || fabsf(full - _pos_hi) < PI) return full;

  const float twoPi = 6.2831853f;
  float mid = 0.5f * (_pos_lo + _pos_hi);
  float k   = roundf((mid - full) / twoPi);
  if (k == 0.0f) return full;
  return full + k * twoPi;
}

// ============================================================
// 装配标定：把当前位置定为云台 0°
// ============================================================
void FOC_Motor::calibrateZero() {
  _encoder.update();
  calibrateZeroAt(_encoder.getAbsAngle(), true);
}

// abs_mech    : 把"编码器绝对角 = abs_mech"这一刻定为云台 0°
// reset_limit : 是否顺手把软限位重置成 [0,180°]
//   · 'h'        -> 用当前位置，限位重置（装配标定的正常流程）
//   · 'h<弧度>'   -> 用指定的编码器角，限位不动（误标定后的精确恢复）
// ★ 带参数这一路是"事后恢复"用的：零点被 h 误改过之后，
//   靠"把轴搬回原位再按 h"永远差一两度，而这里能精确到 0.0001 rad。
void FOC_Motor::calibrateZeroAt(float abs_mech, bool reset_limit) {
  const int motor = _cfg.index;

  _encoder.setAngleOffset(abs_mech);
  _zero_abs = abs_mech;
  foc_config_save_pos_zero(motor, abs_mech);

  // 换了零点之后 full 的数值会瞬间跳变（比如从 0 变成 +0.87 rad），
  // 所以这里按"新零点"重算一次，并把目标锁在当前位置，
  // 否则位置环会看到"巨大误差"直接顶满电流猛冲一下。
  // （'h' 标定当前位置时 full_now 恰好 = 0，与老行为完全一致）
  const float full_now = _encoder.getFullAngle();

  _g_full      = full_now;
  _target_pos  = full_now;
  _target_vel  = 0.0f;
  _vel_cmd     = 0.0f;
  _traj.active = false;
  _traj.cur    = full_now;
  _pos_smooth  = false;
  _sync_norm   = false;
  _current_pid.reset();
  _velocity_pid.reset();
  _position_pid.reset();

  if (reset_limit) {
    // 限位重置成 [0, 180°]（云台对外坐标）并写 NVS。
    // user_sign=-1 时内部是 [-180°, 0]，所以这里换算后直接写内部值。
    _pos_lo    = (_cfg.user_sign > 0.0f) ? foc_param::POS_LIMIT_LO_DEFAULT
                                         : -foc_param::POS_LIMIT_HI_DEFAULT;
    _pos_hi    = (_cfg.user_sign > 0.0f) ? foc_param::POS_LIMIT_HI_DEFAULT
                                         : -foc_param::POS_LIMIT_LO_DEFAULT;
    _limit_hit = false;
    foc_config_save_pos_limit(motor, _pos_lo, _pos_hi);
  }

  _mode = MODE_POSITION;   // 标定完直接锁在这里

  if (reset_limit) {
    Serial.printf("[main] m%d ===== 零点标定完成 =====\n", motor);
    Serial.printf("[main]   当前位置 = 0.0 deg（编码器绝对角 %.4f rad，已存 NVS）\n",
                  (double)abs_mech);
    Serial.printf("[main]   软限位 = [0.0, 180.0] deg\n");
    Serial.printf("[main]   现在可以发 '%dp0' / '%dp90' / '%dp180' 验证\n", motor, motor, motor);
  } else {
    Serial.printf("[main] m%d ===== 零点恢复（指定编码器角）=====\n", motor);
    Serial.printf("[main]   新零点 = 编码器 %.4f rad（已存 NVS）\n", (double)abs_mech);
    Serial.printf("[main]   当前位置 = %.2f deg（限位未改动，仍是 [%.0f, %.0f]）\n",
                  (double)(full_now * 57.2957795f), (double)(_pos_lo * 57.2957795f),
                  (double)(_pos_hi * 57.2957795f));
    Serial.printf("[main]   接着用 'l%d' 把上限改回去\n", motor);
  }
}

// ============================================================
// 轨迹队列
// ============================================================
bool FOC_Motor::push(float pos, float speed) {
  return push(pos, speed, 0.0f);
}

bool FOC_Motor::push(float pos, float speed, float dwell_ms) {
  if (_q_count >= foc_param::QUEUE_SIZE) return false;
  // pos 是对外坐标 → 内部坐标；speed 是速率(标量)，不翻符号
  pos = _cfg.user_sign * pos;
  int tail = (_q_head + _q_count) % foc_param::QUEUE_SIZE;
  _queue[tail].pos   = pos;
  _queue[tail].speed = speed;
  _queue[tail].norm  = false;      // 普通命令（避免复用槽位残留上次的 norm 标志）
  _queue[tail].dwell = (dwell_ms > 0.0f) ? dwell_ms : 0.0f;
  _q_count++;
  _q_total = _q_count;          // 本轮总条数（打印 #n/总数 用）
  _q_done_reported = false;
  return true;
}

bool FOC_Motor::pushNorm(float from, float dist, float span, float vmax) {
  if (_q_count >= foc_param::QUEUE_SIZE) return false;
  int tail = (_q_head + _q_count) % foc_param::QUEUE_SIZE;
  _queue[tail].pos   = from + dist;
  _queue[tail].speed = vmax;
  _queue[tail].norm  = true;
  _queue[tail].dwell = 0.0f;   // 槽位复用，必须显式清零
  _queue[tail].from  = from;
  _queue[tail].dist  = dist;
  _queue[tail].span  = (span > 1e-6f) ? span : 1.0f;
  _q_count++;
  _q_total = _q_count;
  _q_done_reported = false;
  return true;
}

void FOC_Motor::queueStart() {
  _encoder.update();
  // 归一化：轨迹起点必须和位置环同一个坐标系，否则累积角跨圈时
  // （比如实测 -280° 而物理在 79.6°）起点与 _target_pos 差整整一圈，
  // 位置环第一拍就是 360° 误差 → 直接冲限位。
  float full = _nearFull(_encoder.getFullAngle());

  _traj.active = false;
  _traj.cur    = full;
  _target_pos  = full;
  _pos_smooth  = false;
  _sync_norm   = false;
  _dwell_until = 0;      // 新跑一轮，清掉上一轮的停顿计时
  _cur_dwell_ms = 0.0f;

  _current_pid.reset();
  _velocity_pid.reset();
  _position_pid.reset();
  _mode = MODE_TRAJECTORY;
}

void FOC_Motor::queueStop() {
  _q_count = 0;
  _q_total = 0;
  _traj.active = false;
  _pos_smooth  = false;
  _q_done_reported = false;
  _dwell_until  = 0;
  _cur_dwell_ms = 0.0f;

  _mode = MODE_VELOCITY;
  _target_vel = 0;
  _target_pos = 0;

  _current_pid.reset();
  _velocity_pid.reset();
  _position_pid.reset();
}

// ============================================================
// 标定模式 / NVS
// ============================================================
void FOC_Motor::toggleAlignMode() {
  _align_always = !_align_always;
  foc_config_save_align_mode(_cfg.index, _align_always);
  Serial.printf("[main] motor %d align_mode -> %s\n", _cfg.index,
                _align_always ? "ALWAYS (dev: 每次上电对齐)"
                              : "STORED (prod: 有存档则跳过对齐)");
}

void FOC_Motor::clearNvs() const {
  foc_config_clear(_cfg.index);
  Serial.printf("[main] motor %d NVS cleared, will align next boot\n", _cfg.index);
}

// ============================================================
// 遥测打包
// ============================================================
MotorTelem FOC_Motor::telemetry(bool measure) {
  MotorTelem t;
  t.mode  = _mode;
  t.fault = foc_protection_get_fault(_cfg.index);

  float vel_out, iq_out;
  if (measure) {
    // 量化测试：用未滤波原始值 + 一阶平滑，看真实动态
    _mv = 0.5f * _mv + 0.5f * _g_vel_raw;
    _mi = 0.5f * _mi + 0.5f * _g_iq_raw;
    vel_out = _mv;
    iq_out  = _mi;
  } else {
    vel_out = _g_velocity;
    iq_out  = _g_iq;
  }

  if (_mode == MODE_POSITION || _mode == MODE_TRAJECTORY) t.target = _target_pos;
  else if (_mode == MODE_CURRENT)                        t.target = _target_iq;
  else                                                   t.target = _target_vel;

  t.position = _g_full;
  t.velocity = vel_out;
  t.iq       = iq_out;
  t.iq_cmd   = _g_iq_cmd;
  t.vel_cmd  = _vel_cmd;
  return t;
}
