#include "foc_config.h"
#include <Preferences.h>
#include "foc_pwm.h"   // NUM_MOTORS / MOTOR_0 / MOTOR_1

// 两个电机各用一个独立的 NVS 命名空间，避免键名冲突
static Preferences prefs0;
static Preferences prefs1;

// 按电机索引取对应的 Preferences 对象
static Preferences& P(int motor) {
  return (motor == MOTOR_0) ? prefs0 : prefs1;
}

void foc_config_init() {
  prefs0.begin("foc0", false);   // 电机 0 的命名空间
  prefs1.begin("foc1", false);   // 电机 1 的命名空间
  Serial.println("[config] NVS init done (foc0 / foc1)");
}

void foc_config_save_zero_angle(int motor, float angle) {
  P(motor).putFloat("zero_ang", angle);
  Serial.printf("[config] motor %d saved zero_angle = %.4f\n", motor, angle);
}

float foc_config_load_zero_angle(int motor) {
  if (!P(motor).isKey("zero_ang")) {
    return NAN;   // 没保存过
  }
  return P(motor).getFloat("zero_ang", NAN);
}

// ★ 零电角度必须和"算它时用的换相方向 dir"绑定保存。
//   零位的定义是 zero = dir·PP·mech，dir 一变，同一个 zero 数值就配不上新方向
//   （角度关系整体错位）→ 换相错乱 → 通着电不出力、全变铜损发热。
//   把这个坑变成自动检测：存的时候记下 dir，取的时候比对，不一致就强制重新对齐。
void foc_config_save_zero_dir(int motor, int dir) {
  P(motor).putChar("zero_dir", (int8_t)dir);
}

// 返回 0 = 从没存过（调用方把"没存过"当作"不必校验"）
int foc_config_load_zero_dir(int motor) {
  return (int)P(motor).getChar("zero_dir", (int8_t)0);
}

void foc_config_save_align_mode(int motor, bool always_align) {
  P(motor).putUChar("alw_align", always_align ? 1 : 0);
  Serial.printf("[config] motor %d saved align_mode = %s\n",
                motor, always_align ? "ALWAYS(dev)" : "STORED(prod)");
}

bool foc_config_load_align_mode(int motor) {
  // 默认 1 = 开发模式（每次上电对齐），最安全
  return P(motor).getUChar("alw_align", 1) != 0;
}

void foc_config_save_pos_limit(int motor, float lo, float hi) {
  P(motor).putFloat("lim_lo", lo);
  P(motor).putFloat("lim_hi", hi);
  Serial.printf("[config] motor %d saved pos_limit = [%.1f, %.1f] deg\n",
                motor, lo * 57.2958f, hi * 57.2958f);
}

bool foc_config_load_pos_limit(int motor, float& lo, float& hi) {
  if (!P(motor).isKey("lim_lo") || !P(motor).isKey("lim_hi")) return false;
  lo = P(motor).getFloat("lim_lo", lo);
  hi = P(motor).getFloat("lim_hi", hi);
  return true;
}

void foc_config_save_pos_zero(int motor, float abs_mech) {
  P(motor).putFloat("pos_zero", abs_mech);
  Serial.printf("[config] motor %d saved pos_zero = %.4f rad (云台 0° 的编码器角)\n",
                motor, abs_mech);
}

float foc_config_load_pos_zero(int motor) {
  if (!P(motor).isKey("pos_zero")) return NAN;
  return P(motor).getFloat("pos_zero", NAN);
}

void foc_config_clear(int motor) {
  P(motor).remove("zero_ang");
  Serial.printf("[config] motor %d NVS zero_angle cleared\n", motor);
}
