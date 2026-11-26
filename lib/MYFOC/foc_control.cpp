// ============================================================
// 双电机协调层
//
// 结构说明（重构后）：
//   foc_motor.h / .cpp   —— FOC_Motor 类：一台电机的全部状态 + 三环 + 保护 + 诊断
//   foc_control.h / .cpp —— 本文件：持有 2 台实例、管初始化顺序、跑循环、拼遥测
//
// 重构原则：纯结构重构，控制行为零改变。
//   参数、滤波系数、打印内容、保护调用时机全部与原 foc_control.cpp 一致。
//   旧 API 一律保留为一行转发，foc_serial.cpp / foc_can.cpp 不需要任何改动。
// ============================================================

#include "foc_control.h"

#include "foc_config.h"
#include "foc_protection.h"
#include "foc_pwm.h"

#include <Wire.h>

// ============================================================
// 引脚定义（和 DengFOCtext10 完全一致）
// ============================================================
namespace {

// ---- 电机 0 (M0) ----
constexpr int M0_PWM_A = 15;
constexpr int M0_PWM_B = 16;
constexpr int M0_PWM_C = 17;
constexpr int M0_EN    = 10;
constexpr int M0_SDA   = 11;
constexpr int M0_SCL   = 12;
constexpr int M0_ADC_A = 4;
constexpr int M0_ADC_B = 5;

// ---- 电机 1 (M1) ----
constexpr int M1_PWM_A = 18;
constexpr int M1_PWM_B = 8;
constexpr int M1_PWM_C = 9;
constexpr int M1_EN    = 10;   // ★ 硬件上两颗电机共用同一个 EN（DengFOC 原版也是 enable=10）
constexpr int M1_SDA   = 13;
constexpr int M1_SCL   = 14;
constexpr int M1_ADC_A = 6;
constexpr int M1_ADC_B = 7;

// ---- 电气参数 ----
constexpr int   MOTOR_PP = 7;      // 极对数
constexpr int   MOTOR_DIR = 1;     // 编码器方向
constexpr float VBUS = 12.6f;

}  // namespace

// ============================================================
// 两台电机的实例
// 想加第三台？在这里加一个元素 + 一份 MotorConfig + 下面的循环自动跟上。
// ============================================================
static FOC_Motor g_motors[NUM_MOTORS];

// 测量模式是"遥测行为"，全局开关，不属于某台电机
static bool g_measure = false;

// 诊断模式：关闭遥测打印，只留 [TRAJ]/[ENC]/[BIGERR] 诊断行。
// 用途：抓偶发 bug 时，遥测 5ms 两行刷太快会把诊断行冲掉，关掉遥测就能看清。
static bool g_diag = false;

// 控制环实际频率（Hz），由 foc_control_update 每秒统计一次，'?' 里打印
static float g_ctrl_hz = 0.0f;
float foc_control_get_hz() { return g_ctrl_hz; }

// ============================================================
// 面向对象入口
// ============================================================
FOC_Motor& foc_motor(int motor) {
  if (motor < 0 || motor >= NUM_MOTORS) motor = 0;
  return g_motors[motor];
}

// ============================================================
// 初始化
// ============================================================
void foc_control_init() {
  // ---- 先把 NVS 和保护状态准备好，再初始化电机 ----
  foc_config_init();
  foc_protection_init();

  // ---- 阶段 1：两台电机的硬件 + 电流零偏校准 + 三环 PID（都不使能）----
  // 顺序很重要：两颗电机都在"静止 + 不使能"时做 ADC 零偏校准，
  // 否则一路绕组通电会影响另一路的采样基准。
  MotorConfig cfg0;
  cfg0.index      = MOTOR_0;
  cfg0.pin_a      = M0_PWM_A;
  cfg0.pin_b      = M0_PWM_B;
  cfg0.pin_c      = M0_PWM_C;
  cfg0.pin_en     = M0_EN;
  cfg0.sda        = M0_SDA;
  cfg0.scl        = M0_SCL;
  cfg0.adc_a      = M0_ADC_A;
  cfg0.adc_b      = M0_ADC_B;
  cfg0.wire       = &Wire;
  cfg0.pole_pairs = MOTOR_PP;
  cfg0.dir        = MOTOR_DIR;
  cfg0.vbus       = VBUS;
  cfg0.vel_kp     = foc_param::VEL_KP_YAW;    // 竖直 yaw：大惯量，靠高 P 抑制持续摆动
  g_motors[MOTOR_0].begin(cfg0);

  MotorConfig cfg1;
  cfg1.index      = MOTOR_1;
  cfg1.pin_a      = M1_PWM_A;
  cfg1.pin_b      = M1_PWM_B;
  cfg1.pin_c      = M1_PWM_C;
  cfg1.pin_en     = M1_EN;
  cfg1.sda        = M1_SDA;
  cfg1.scl        = M1_SCL;
  cfg1.adc_a      = M1_ADC_A;
  cfg1.adc_b      = M1_ADC_B;
  cfg1.wire       = &Wire1;    // 第二路 I2C，避免和 M0 的 AS5600 抢地址
  cfg1.pole_pairs = MOTOR_PP;
  cfg1.dir        = MOTOR_DIR;
  cfg1.vbus       = VBUS;
  // ★ pitch 轴惯量小得多（只有笔管），速度环 P 用 0.15 会把测速噪声放大成
  //   "到点后极小幅度高频细震"，所以单独给 0.05。'1w0.15' 可以现场改回去对比。
  cfg1.vel_kp     = foc_param::VEL_KP_PITCH;

  // ★ M1 编码器方向与对外坐标相反：内部累积角增大 = 物理顺时针。
  //   用户坐标系要"角度增大 = 逆时针"，所以对外翻符号（只翻 p/v/q/l/显示，
  //   三环与换相仍走内部坐标）。改这一行即可，不要动 cfg1.dir / 电角度零点。
  cfg1.user_sign  = -1.0f;
  g_motors[MOTOR_1].begin(cfg1);

  // ---- 阶段 2：逐个使能 + 读 NVS + （按需）对齐 ----
  for (int i = 0; i < NUM_MOTORS; i++) {
    g_motors[i].start();
  }

  // ---- 启动帮助 ----
  Serial.println("[main] Ready.");
  Serial.println("[main] FW-BUILD 2026-11-18-D: 跨圈坐标归一化(治走大圈撞限位) + 电流2次过采样");
  Serial.printf("[main] 速度环 P: m0=%.3f m1=%.3f, vel filter Tf=%.3fs\n",
                (double)g_motors[0].getVelKp(), (double)g_motors[1].getVelKp(),
                (double)foc_param::VEL_FILT_TF);
  Serial.println("[main] DIAG-BUILD 2026-11-03: 本固件含诊断日志，用串口监视器看 [TRAJ]/[ENC]/[BIGERR]/[STUCK]/[LIMIT]");
  // ---- 命令按用途分组，避免"一长串不知道哪个能用" ----
  Serial.println("[main] ================= 命令表 =================");
  Serial.println("[main] 【日常】所有运动命令的「电机号」写最前面，不写=电机0");
  Serial.println("[main]   'p90'        -> 去 90°（单位：度），例 '0p0' '1p180'");
  Serial.println("[main]   'q0,0,30;0,180,30;1,90,30' -> 轨迹队列跑一条路径");
  Serial.println("[main]                  （格式：电机号,目标度,速度 rad/s; 用 ; 分段）");
  Serial.println("[main]   'g x,y,z[,v]'-> 指向命令：给空间方向自动算两轴角度");
  Serial.println("[main]   'e'          -> 急停（清空该电机队列）");
  Serial.println("[main] 【标定/限位】装配后做一次，写 NVS 永久保存");
  Serial.println("[main]   'h'          -> 把当前位置定为该轴 0°（限位重置 0~180°）");
  Serial.println("[main]   'h5.0621'    -> 把编码器 5.0621 rad 处定为 0°（限位不动，零点误改时恢复用）");
  Serial.println("[main]   'l'          -> 查限位；'l170'=[0,170°]；'l-10,170' 非对称");
  Serial.println("[main]   'z yaw,pitch'-> 指向命令的零位偏置；'y' 翻 yaw 方向");
  Serial.println("[main] 【查看/救急】");
  Serial.println("[main]   '?'          -> 两轴状态（角度、误差、速度、电流、故障）");
  Serial.println("[main]   'r'          -> 清故障；'d' 诊断模式；'m' 单次测量");
  Serial.println("[main] 【CANopen 学习】CAN 走 0x000/0x600/0x700，与旧 0x100 帧不冲突");
  Serial.println("[main]   't'          -> 板内回环自检（假主站，5 项 SDO/NMT/心跳）");
  Serial.println("[main]   'or' / 'or2'  -> 你当主站问 M0/M1「现在在多少度」");
  Serial.println("[main] 【内环调试——正常使用不用管】");
  Serial.println("[main]   'i0.3'       -> 电流模式（直接给 iq，仅测硬件用）");
  Serial.println("[main]   'v10'        -> 速度模式（单位 rad/s）");
  Serial.println("[main]   'w' 'k' 'j'  -> 速度环P / 位置环P,Kd / 力矩上限A");
  Serial.println("[main]   'c' 's' 'x' 'a' -> 电角度重标定/查档/清档/对齐模式");
  Serial.println("[main] ------------------------------------------------");
  Serial.println("[main] 遥测每行 14 列：列1~7=电机0，列8~14=电机1");
  Serial.println("[main] 每电机 7 列：模式, 目标值, 实际位置, 实际速度, 实际电流, 电流指令, 故障码");
  Serial.println("[main] 模式码：0=电流 / 1=速度 / 2=位置 / 3=轨迹；目标值单位 A / rad/s / rad");
}

// ============================================================
// 每 loop：所有电机各跑一遍三环
// ============================================================
void foc_control_update() {
  // 控制环实际频率统计：debug 用，成本只有一次 micros()+自增。
  // 这个数字很关键 —— 记忆里的老教训是「loop 里加 delay 会把频率压到 270Hz，
  // 同样的 PID 参数每步积分量暴增 → 离散化失稳 → 震」。同理，任何让单次 update
  // 变慢的改动（比如电流采样过采样次数）都会偷偷降频，必须能看到。
  static uint32_t cnt = 0;
  static uint32_t t0  = 0;
  cnt++;
  uint32_t now_us = micros();
  if (now_us - t0 >= 1000000UL) {
    g_ctrl_hz = (t0 == 0) ? 0.0f : (cnt * 1000000.0f / (float)(now_us - t0));
    cnt = 0;
    t0  = now_us;
  }

  for (int i = 0; i < NUM_MOTORS; i++) {
    g_motors[i].update();
  }
}

// ============================================================
// 打印遥测：一行 14 列，语义固定不随模式变
//   列 1~7  = 电机0，列 8~14 = 电机1
//   每电机 7 列：模式码, 目标值, 实际位置, 实际速度, 实际电流, 电流指令, 故障码
//     模式码：0=电流 / 1=速度 / 2=位置 / 3=轨迹
//     目标值随模式变：电流(A) / 速度(rad/s) / 位置(rad)
// ============================================================
void foc_control_print() {
  // 诊断模式：不打遥测，串口上只剩诊断行，肉眼能看清 [TRAJ]/[ENC]/[BIGERR]
  if (g_diag) return;

  // ★ 2026-11-18：改成「按时间」分频，不再按 loop 计数。
  //   按 loop 计数时控制环频率一变（比如开 m 高频打印把 loop 拖到 130Hz），
  //   打印频率就跟着乱，而且一行 14 列 ~60 字节 @115200 = 5.2ms，
  //   每 loop 打一次会直接把控制环压到 100Hz 量级 → PID 失稳，测出来的"抖动"
  //   是打印自己造成的假象。定时打印则与 loop 频率解耦。
  static uint32_t last_ms = 0;
  const uint32_t period_ms = g_measure ? 20 : 100;   // 测量模式 50Hz / 平时 10Hz
  uint32_t now_ms = millis();
  if (now_ms - last_ms < period_ms) return;
  last_ms = now_ms;

  // 先收集所有电机的数据，再一次打印成一行，避免多行交错导致 VOFA 波形串扰
  MotorTelem t[NUM_MOTORS];
  for (int i = 0; i < NUM_MOTORS; i++) {
    t[i] = g_motors[i].telemetry(g_measure);
    // 对外坐标：M1 user_sign=-1，波形/遥测也跟着用户坐标系走，
    // 否则波形方向和实际动作对不上，越看越乱。
    const float s = g_motors[i].userSign();
    t[i].target   *= s;
    t[i].position *= s;
    t[i].velocity *= s;
    t[i].vel_cmd  *= s;
  }

  Serial.printf(
    "%d,%.4f,%.4f,%.4f,%.4f,%.4f,%d,"
    "%d,%.4f,%.4f,%.4f,%.4f,%.4f,%d\n",
    t[0].mode, t[0].target, t[0].position, t[0].velocity, t[0].iq, t[0].iq_cmd, t[0].fault,
    t[1].mode, t[1].target, t[1].position, t[1].velocity, t[1].iq, t[1].iq_cmd, t[1].fault);
}

// ============================================================
// 测量模式开关
// ============================================================
void foc_control_set_measure(bool on) {
  g_measure = on;
  if (on) {
    Serial.println("[main] measure mode ON  -> 未滤波反馈 + 每次 loop 都打印");
    Serial.println("[main]   波特率 115200；测完发 m 关掉");
  } else {
    Serial.println("[main] measure mode OFF -> 恢复滤波反馈 + 降频打印");
  }
}

bool foc_control_get_measure() { return g_measure; }

// ============================================================
// 诊断模式开关（抓偶发 bug 用：关遥测，只留诊断行）
// ============================================================
void foc_control_set_diag(bool on) {
  g_diag = on;
  if (on) {
    Serial.println("[DIAG] 诊断模式 ON：遥测已关闭，串口只打印 [TRAJ]/[ENC]/[BIGERR]/[CO] 等事件行");
    Serial.println("[DIAG] 看 CANopen 帧时开着这个最清爽；测完再发 d 恢复遥测");
  } else {
    Serial.println("[DIAG] 诊断模式 OFF：恢复遥测打印");
  }
}

bool foc_control_get_diag() { return g_diag; }

// ============================================================
// 旧 API：全部一行转发到 FOC_Motor 成员方法
// ============================================================
void  foc_set_mode(int motor, int mode)       { foc_motor(motor).setMode(mode); }
void  foc_set_target_iq(int motor, float iq)  { foc_motor(motor).setTargetIq(iq); }
void  foc_set_target_vel(int motor, float v)  { foc_motor(motor).setTargetVel(v); }
void  foc_set_target_pos(int motor, float v)  { foc_motor(motor).setTargetPos(v); }
void  foc_set_smooth(int motor, bool en)     { foc_motor(motor).setSmooth(en); }

int   foc_get_mode(int motor)                 { return foc_motor(motor).mode(); }
float foc_get_target_iq(int motor)            { return foc_motor(motor).targetIq(); }
float foc_get_target_vel(int motor)           { return foc_motor(motor).targetVel(); }
float foc_get_target_pos(int motor)           { return foc_motor(motor).targetPos(); }

float foc_control_get_full(int motor)         { return foc_motor(motor).fullAngle(); }

// 归一化 + 用户坐标（乘 user_sign）的角度。
// 逆解（g 命令）必须用这个：
//   · 不归一化 → 编码器漏计整圈时拿到 -280°，与 0~180° 的限位比较会钳出大圈
//   · 不乘 sign  → M1(user_sign=-1) 的内部角是负的，跟 0~180° 的用户限位根本不是一套坐标
float foc_control_get_full_near(int motor) {
  FOC_Motor& m = foc_motor(motor);
  return m.userSign() * m.fullAngleNear();
}

// 对外坐标符号：显示角度/速度时乘它，保证"看到的就是用户坐标"
float foc_get_user_sign(int motor)            { return foc_motor(motor).userSign(); }
float foc_control_get_velocity(int motor)     { return foc_motor(motor).velocity(); }
float foc_control_get_iq(int motor)           { return foc_motor(motor).iq(); }

void  foc_control_recalibrate(int motor)      { foc_motor(motor).recalibrate(); }
void  foc_control_clear_nvs(int motor)        { foc_motor(motor).clearNvs(); }

void  foc_control_toggle_align_mode(int motor) { foc_motor(motor).toggleAlignMode(); }
bool  foc_control_get_align_mode(int motor)    { return foc_motor(motor).alignAlways(); }

void  foc_set_pos_kp(int motor, float k)        { foc_motor(motor).setPosKp(k); }
float foc_get_pos_kp(int motor)                 { return foc_motor(motor).getPosKp(); }
void  foc_set_pos_kd(int motor, float d)        { foc_motor(motor).setPosKd(d); }
void  foc_set_vel_kp(int motor, float k)        { foc_motor(motor).setVelKp(k); }
float foc_get_vel_kp(int motor)                 { return foc_motor(motor).getVelKp(); }
void  foc_calibrate_zero(int motor)             { foc_motor(motor).calibrateZero(); }
void  foc_calibrate_zero_at(int motor, float abs_mech) {
  if (motor < 0 || motor >= NUM_MOTORS) return;
  foc_motor(motor).calibrateZeroAt(abs_mech, false);
}
float foc_get_zero_offset(int motor)            { return foc_motor(motor).zeroOffset(); }
float foc_get_pos_kd(int motor)                 { return foc_motor(motor).getPosKd(); }

void  foc_set_vel_iq_limit(int motor, float a){ foc_motor(motor).setVelIqLimit(a); }
float foc_get_vel_iq_limit(int motor)          { return foc_motor(motor).getVelIqLimit(); }

void  foc_set_pos_limit(int motor, float lo, float hi) { foc_motor(motor).setPosLimit(lo, hi); }
float foc_get_pos_limit_lo(int motor)  { return foc_motor(motor).posLimitLo(); }
float foc_get_pos_limit_hi(int motor)  { return foc_motor(motor).posLimitHi(); }
bool  foc_get_pos_limit_hit(int motor) { return foc_motor(motor).posLimitHit(); }

// ============================================================
// 轨迹指令队列
// ============================================================
bool foc_queue_push(int motor, float pos, float speed, float dwell_ms) {
  if (motor < 0 || motor >= NUM_MOTORS) return false;
  return g_motors[motor].push(pos, speed, dwell_ms);
}

bool foc_queue_push_norm(int motor, float from, float dist, float span, float vmax) {
  if (motor < 0 || motor >= NUM_MOTORS) return false;
  return g_motors[motor].pushNorm(from, dist, span, vmax);
}

int foc_queue_count(int motor) {
  if (motor < 0 || motor >= NUM_MOTORS) return 0;
  return g_motors[motor].queueCount();
}

void foc_queue_stop(int motor) {
  if (motor < 0 || motor >= NUM_MOTORS) return;
  g_motors[motor].queueStop();
}

void foc_queue_start(int motor) {
  if (motor < 0 || motor >= NUM_MOTORS) return;
  g_motors[motor].queueStart();
}

// ============================================================
// 保护查询 / 清除：一行转发，供 CANopen 使用
//   为什么不让 canopen.cpp 直接 include foc_protection.h：
//   层次上 CANopen 只该认识"电机的对外能力"，不该知道保护模块叫什么、
//   有几个阈值。以后换实现（比如加温度保护）上层代码一行都不用改。
// ============================================================
int foc_get_fault(int motor) {
  if (motor < 0 || motor >= NUM_MOTORS) return 0;
  return foc_protection_get_fault(motor);
}

void foc_clear_fault(int motor) {
  if (motor < 0 || motor >= NUM_MOTORS) return;
  foc_protection_clear(motor);
}

void foc_notify_cmd(int motor) {
  if (motor < 0 || motor >= NUM_MOTORS) return;
  foc_protection_notify_cmd(motor);
}
