#include "foc_pwm.h"
#include "foc_math.h"

// 每个电机的三相 PWM 引脚 + 使能引脚（用数组存，下标 = 电机索引）
static int _pinA[NUM_MOTORS], _pinB[NUM_MOTORS], _pinC[NUM_MOTORS], _pinEN[NUM_MOTORS];
static uint32_t _freq;
static uint8_t  _res;

// 每个电机占 3 个 LEDC 通道：电机 0 用 0/1/2，电机 1 用 3/4/5
// ESP32-S3 高速 LEDC 有 8 个通道（0~7），6 路 PWM 够用。
static const int _ch[NUM_MOTORS][3] = {
    {0, 1, 2},
    {3, 4, 5},
};

void pwm_init(int motor, int pinA, int pinB, int pinC, int pinEN,
              uint32_t freq, uint8_t resolution) {
  if (motor < 0 || motor >= NUM_MOTORS) return;

  _pinA[motor] = pinA;
  _pinB[motor] = pinB;
  _pinC[motor] = pinC;
  _pinEN[motor] = pinEN;
  _freq = freq;
  _res  = resolution;

  // -------- 配置三路 LEDC PWM --------
  ledcSetup(_ch[motor][0], _freq, _res);
  ledcSetup(_ch[motor][1], _freq, _res);
  ledcSetup(_ch[motor][2], _freq, _res);

  // -------- 把引脚绑定到 LEDC 通道 --------
  ledcAttachPin(_pinA[motor], _ch[motor][0]);
  ledcAttachPin(_pinB[motor], _ch[motor][1]);
  ledcAttachPin(_pinC[motor], _ch[motor][2]);

  // -------- 使能引脚 --------
  pinMode(_pinEN[motor], OUTPUT);
  digitalWrite(_pinEN[motor], LOW);   // 默认失能，安全

  // -------- 三路 PWM 先输出 50%：三相等电位 = coast（自由滑行）--------
  // ★ 原来这里写 0：三相全 0% 占空比 = 三个下桥全开 = 绕组被短接到地 = 短路制动。
  //   pwm_enable() 只是拉 EN，所以"使能的那一瞬间"电机处在刹车状态，
  //   装了云台有惯量/重力时会被猛刹一下，表现为上电抽动。
  //   改成 50/50/50 后三相对中性点电压相同 → 不出力、也不刹车，平滑接管。
  const uint32_t half = ((1u << _res) - 1u) / 2u;
  ledcWrite(_ch[motor][0], half);
  ledcWrite(_ch[motor][1], half);
  ledcWrite(_ch[motor][2], half);

  Serial.printf("[foc_pwm] motor %d init done\n", motor);
}

void pwm_enable(int motor) {
  if (motor < 0 || motor >= NUM_MOTORS) return;
  digitalWrite(_pinEN[motor], HIGH);
}

void pwm_disable(int motor) {
  if (motor < 0 || motor >= NUM_MOTORS) return;
  digitalWrite(_pinEN[motor], LOW);
}

void pwm_set_duty(int motor, float dc_a, float dc_b, float dc_c) {
  if (motor < 0 || motor >= NUM_MOTORS) return;

  // 限制到 0~1，防止越界
  dc_a = constrain(dc_a, 0.0f, 1.0f);
  dc_b = constrain(dc_b, 0.0f, 1.0f);
  dc_c = constrain(dc_c, 0.0f, 1.0f);

  // 转成 LEDC 的数值（8 位 -> 255）
  uint32_t max_duty = (1 << _res) - 1;
  ledcWrite(_ch[motor][0], (uint32_t)(dc_a * max_duty));
  ledcWrite(_ch[motor][1], (uint32_t)(dc_b * max_duty));
  ledcWrite(_ch[motor][2], (uint32_t)(dc_c * max_duty));
}

// ------------------------------------------------------------
// 开环力矩输出（SVPWM）
// 1. 逆 Park：Ualpha = -Uq*sin(angle_el)，Ubeta = Uq*cos(angle_el)
// 2. 逆 Clarke：Ua/Ub/Uc（不加偏置，相对中性点）
// 3. 零序注入：uoff = -(umax+umin)/2
// 4. 加 Vbus/2 偏置转成占空比
// ------------------------------------------------------------
void pwm_set_torque_openloop(int motor, float Uq, float angle_el, float vbus) {
  Uq = constrain(Uq, -vbus * 0.5f, vbus * 0.5f);
  angle_el = normalizeAngle(angle_el);

  // 1. 逆 Park 变换：Uq -> Ualpha, Ubeta
  float Ualpha = -Uq * sinf(angle_el);
  float Ubeta  =  Uq * cosf(angle_el);

  // 2. 逆 Clarke 变换：不加偏置，只算相对于中性点的电压
  float Ua = Ualpha;
  float Ub = (sqrtf(3.0f) * Ubeta - Ualpha) * 0.5f;
  float Uc = (-Ualpha - sqrtf(3.0f) * Ubeta) * 0.5f;

  // 3. SVPWM 核心：零序注入（共模电压）
  float umax = max(Ua, max(Ub, Uc));
  float umin = min(Ua, min(Ub, Uc));
  float uoff = -(umax + umin) * 0.5f;

  // 4. 加上 Vbus/2 偏置，转成占空比
  float dc_a = 0.5f + (Ua + uoff) / vbus;
  float dc_b = 0.5f + (Ub + uoff) / vbus;
  float dc_c = 0.5f + (Uc + uoff) / vbus;

  // 5. 输出到 PWM
  pwm_set_duty(motor, dc_a, dc_b, dc_c);
}
