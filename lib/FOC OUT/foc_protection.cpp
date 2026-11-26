#include "foc_protection.h"
#include "foc_pwm.h"       // pwm_enable / pwm_disable + NUM_MOTORS
#include "foc_control.h"   // MODE_CURRENT / MODE_VELOCITY / MODE_POSITION 定义在这里

// 每个电机一套独立的保护状态（数组下标 = 电机索引）
struct ProtectionState {
  int      fault_code;
  uint32_t stall_start_ts;
  float    stall_pos_ref;
  uint32_t pos_err_start_ts;
  uint32_t last_cmd_ts;
  uint32_t boot_ts;
  int      oc_count;
};
static ProtectionState ps[NUM_MOTORS];

#define STARTUP_GRACE_MS   2000
#define OC_DEBOUNCE_CNT    20    // 1kHz 控制环下 ≈ 20ms

// ------------------------------------------------------------
void foc_protection_init() {
  for (int m = 0; m < NUM_MOTORS; m++) {
    ps[m].fault_code     = FAULT_NONE;
    ps[m].stall_start_ts = 0;
    ps[m].stall_pos_ref  = 0;
    ps[m].pos_err_start_ts = 0;
    ps[m].last_cmd_ts    = millis();
    ps[m].boot_ts        = millis();
    ps[m].oc_count       = 0;
  }
}

// ------------------------------------------------------------
bool foc_protection_is_fault(int motor) {
  return ps[motor].fault_code != FAULT_NONE;
}

// ------------------------------------------------------------
int foc_protection_get_fault(int motor) {
  return ps[motor].fault_code;
}

// ------------------------------------------------------------
// 进入故障：停掉该电机的力矩输出，记录故障码
// ★ 注意：两台电机共用同一个 EN 引脚（foc_control.cpp 里 M0_EN == M1_EN == 10）。
//   如果这里调 pwm_disable(motor)，会把 GPIO10 拉低 → 另一台电机同时失去力矩
//   → 另一台停在半路 → 3 秒后也报位置超差，形成"一台故障、两台都报"的连锁故障。
//   所以改成：只把本机三相占空比设为 50%（三相等电位 = 零电压 = 自由滑行），
//   不碰共用 EN，另一台继续正常工作。
// ------------------------------------------------------------
static void enterFault(int motor, int code) {
  ps[motor].fault_code |= code;
  pwm_set_duty(motor, 0.5f, 0.5f, 0.5f);   // 本机零电压滑行，不影响另一台
  Serial.printf("[FAULT] motor %d code=0x%02X (PWM->coast)\n", motor, ps[motor].fault_code);
}

// ------------------------------------------------------------
// 每 loop 调用的保护检查
// ------------------------------------------------------------
void foc_protection_update(int motor, float iq, float velocity, float pos_error,
                           float cmd_vel, float pos, int mode) {
  ProtectionState& s = ps[motor];
  if (s.fault_code != FAULT_NONE) return;   // 已故障，不再检查

  // 启动宽限期：上电后的对齐阶段 + 初始启动瞬间，电流/速度会有较大过冲
  bool in_grace = (millis() - s.boot_ts) < STARTUP_GRACE_MS;

  // --- 过流（故障码 = 1）：20ms 防抖 ---
  if (!in_grace && fabsf(iq) > OC_THRESHOLD) {
    if (++s.oc_count >= OC_DEBOUNCE_CNT) {
      enterFault(motor, FAULT_OVERCURRENT);
      return;
    }
  } else {
    s.oc_count = 0;
  }

  // 堵转 / 位置超差 在宽限期内也不判
  if (!in_grace) {
    // --- 堵转（故障码 = 2）：仅速度模式 + 位移判据 ---
    bool cmd_moving = (mode == MODE_VELOCITY) && (fabsf(cmd_vel) > STALL_VELOCITY);
    if (cmd_moving && fabsf(iq) > STALL_CURRENT) {
      if (s.stall_start_ts == 0) {
        s.stall_start_ts = millis();
        s.stall_pos_ref  = pos;
      }
      if (fabsf(pos - s.stall_pos_ref) > STALL_MIN_MOVE) {
        // 它在动，只是慢：重置窗口
        s.stall_start_ts = millis();
        s.stall_pos_ref  = pos;
      } else if (millis() - s.stall_start_ts > STALL_TIME_MS) {
        Serial.printf("[FAULT] motor %d stall: cmd=%.2f vel=%.2f iq=%.2f dpos=%.3f\n",
                      motor, cmd_vel, velocity, (double)iq, (double)(pos - s.stall_pos_ref));
        enterFault(motor, FAULT_STALL);
        return;
      }
    } else {
      s.stall_start_ts = 0;
    }

    // --- 位置超差（故障码 = 4，位置模式 + 轨迹模式） ---
    // 由 POS_ERROR_PROT_ENABLE 总开关控制（foc_protection.h）。
    // 调试阶段默认关闭：这条保护只是"提醒"，会打断测试，但不解决任何问题。
    // 真正要查"转不到位"，看 [SNAP] 那 20 帧数据。
    if (POS_ERROR_PROT_ENABLE &&
        (mode == MODE_POSITION || mode == MODE_TRAJECTORY) &&
        fabsf(pos_error) > POS_ERROR_LIMIT && fabsf(velocity) < STALL_VELOCITY) {
      if (s.pos_err_start_ts == 0) s.pos_err_start_ts = millis();
      if (millis() - s.pos_err_start_ts > POS_ERROR_TIME_MS) {
        // 报故障前打一行上下文：误差多大、当时速度/电流/速度指令多少、停在哪
        // 用来区分「力矩不够（iq 顶到限幅）」和「控制没给力（iq 很小）」
        Serial.printf("[FAULT] motor %d pos_error: err=%.3f vel=%.3f iq=%.3f "
                      "cmd_vel=%.3f pos=%.3f\n",
                      motor, (double)pos_error, (double)velocity, (double)iq,
                      (double)cmd_vel, (double)pos);
        enterFault(motor, FAULT_POS_ERROR);
        return;
      }
    } else {
      s.pos_err_start_ts = 0;
    }
  }

  // --- 指令超时（故障码 = 8）：只在“命令要走 + 电机真在走”时判 ---
  //   ★ 2026-11-19 从 || 改成 &&。原来只要速度超过 0.5 rad/s 就算“在动”，
  //     结果编码器偶发重同步会甩出一个 113 rad/s 的毛刺（[ENC] vel spike 113.2），
  //     瞬间满足条件 + 长时间没串口命令 → 误报“指令超时”，电机被强行 coast。
  //     改成 && 后：静止保持（cmd_vel=0）永远不会超时，
  //     而“速度模式下上位机真挂了”仍然能抓到（cmd_vel 非 0 且电机在转）。
  //     卡住不转属于堵转（码 2）的职责，不该由超时来管。
  bool active = (fabsf(cmd_vel) > STALL_VELOCITY) && (fabsf(velocity) > STALL_VELOCITY);
  if (active && (millis() - s.last_cmd_ts > CMD_TIMEOUT_MS)) {
    enterFault(motor, FAULT_CMD_TIMEOUT);
    return;
  }
}

// ------------------------------------------------------------
void foc_protection_notify_cmd(int motor) {
  ps[motor].last_cmd_ts = millis();
}

// ------------------------------------------------------------
void foc_protection_clear(int motor) {
  ps[motor].fault_code     = FAULT_NONE;
  ps[motor].stall_start_ts = 0;
  ps[motor].stall_pos_ref  = 0;
  ps[motor].pos_err_start_ts = 0;
  ps[motor].last_cmd_ts    = millis();
  pwm_enable(motor);
  Serial.printf("[main] motor %d FAULT CLEARED, enabled\n", motor);
}
