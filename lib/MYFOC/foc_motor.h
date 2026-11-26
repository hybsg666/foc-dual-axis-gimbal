#ifndef FOC_MOTOR_H
#define FOC_MOTOR_H

#include <Arduino.h>
#include <Wire.h>

#include "foc_current.h"
#include "foc_encoder.h"
#include "foc_pid.h"
#include "foc_trajectory.h"

// ============================================================
// 运行模式
// ============================================================
#define MODE_CURRENT     0
#define MODE_VELOCITY    1
#define MODE_POSITION    2
#define MODE_TRAJECTORY  3   // 轨迹模式：按指令队列 + 梯形速度曲线逐条执行

// ============================================================
// 全局默认参数（每台电机可在 MotorConfig 里单独覆盖）
//
// ★ 量纲澄清（很重要）：DengFOCtext10 里 SET_VEL_PID(0.02,1,0,100000,0.5) 的 0.5 是
//   **电压 0.5V**——因为它的 set_Velocity_Angle 直接 setTorque(速度环输出)，
//   中间没有电流环；0.5V 落在相电阻上只有约 0.05A。
//   本工程在速度环后串了电流环，同一个 0.5 变成了 **0.5A**，是 DengFOC 实际电流的
//   十倍以上。所以这里的 0.5A 已经是"偏猛"的值，不要再往上放——
//   再大就容易把 12V 适配器拉跌 / 触发驱动欠压，表现成"转到一半突然没力气停住"。
// ============================================================
namespace foc_param {

constexpr float IQ_LIMIT       = 2.0f;    // 电流模式目标限幅 A
constexpr float VEL_LIMIT      = 30.0f;   // 位置环输出（速度指令）限幅 rad/s
constexpr float VEL_LIMIT_NEAR = 0.5f;    // 速度环输出（iq 目标）限幅 A
constexpr float VEL_TARGET_MAX = 50.0f;   // 速度模式目标限幅 rad/s
constexpr float POS_NEAR_RAD   = 0.10f;   // 到位判定（仅诊断用）

// ============================================================
// ★★ 速度环 P —— 这是"M0 一直摆、Kd 怎么加都没用"的真正病根 ★★
//
// 为什么 Kd 无效（数学）：
//   vel_cmd = Kp·err − Kd·ω，速度环 iq = Pv·(vel_cmd − ω) + Iv·∫(...)
//   代入后 = Pv·Kp·err − Pv·(1+Kd)·ω + Iv·∫(...)
//   → Kd 只能把"阻尼系数"从 Pv 抬到 Pv·(1+Kd)。
//   → Pv 只有 0.02，Kd 从 0.3 加到 1.0，系数从 0.026 变 0.04，
//     绝对增量 0.014 —— 而速度环的刚度由 Iv=1.0 决定，
//     阻尼比 ζ = Kt·Pv·(1+Kd) / (2·J·ωn) 几乎不动 ⇒ 肉眼看就是"完全没区别"。
//
// 为什么 M0 会摆而 M1 不摆（物理）：
//   M0 是竖直 yaw 轴，质心落在轴线上 → 重力不产生力矩 → 轴系是"纯惯量 + 近乎无摩擦轴承"。
//   速度环 P 只有 0.02 时，ζ ≈ 0.1~0.2，就是一台**无阻尼扭摆**：
//   任何扰动（上电、手碰、编码器噪声）都会激起 1~2Hz 的持续摆动。
//   M1 是 pitch 轴，重力矩让速度环积分器停在一个恒定工作点上"顶着"，反而稳。
//
// 修法：直接把 Pv 提到 0.15（≈7.5 倍），ζ 才真正 > 1。
//   代价：速度噪声(≈0.2rad/s) × 0.15 = 0.03A 电流纹波，相对 0.5A 限幅可以接受。
//   若装好后出现"细密嗡嗡"，说明 Pv 偏大，用 'w' 命令往回收。
constexpr float VEL_KP_BASE = 0.15f;   // 速度环 P 默认值（原 0.02）
constexpr float VEL_KI_BASE = 1.0f;    // 速度环 I（DengFOC 原值，保持）

// ★ 速度环 P 的按轴默认值（见 MotorConfig::vel_kp 的说明）
constexpr float VEL_KP_YAW   = 0.15f;  // M0 竖直轴：带整个悬臂，惯量大，必须靠它压摆
constexpr float VEL_KP_PITCH = 0.05f;  // M1 水平轴：只有笔管，惯量小，P 大了反而细震

// ★ 速度反馈低通时间常数 Tf（秒）
//   测速是"每拍角度差分 / 3.7ms"，AS5600 有 0.088°/LSB 量化，
//   于是速度噪声 ≈ 0.4 rad/s —— 这个噪声 × 速度环 P 直接变成电流纹波，
//   再变成可闻的电磁噪声（PWM 是 30kHz 听不见，听到的是纹波激励的结构振动）。
//
//   Tf 从 0.01 提到 0.025 的效果（tools/sim_veltf.py 实测仿真）：
//     iq 逐拍变化从 0.0147 降到 0.0062（噪声降到 1/2.4），
//     而 M0 的停住时间 0.46s → 0.42s（阻尼没有受损）。
//   再加到 0.04，噪声更小(1/3.8)，但停住时间退化到 0.66s —— 相位滞后开始吃阻尼。
//   所以 0.025 是最佳折中：噪声砍掉六成，1~2Hz 的摆动只滞后 13°。
constexpr float VEL_FILT_TF  = 0.025f;

// ★ 位置环死区（度）
//   位置环的 P=1 是**以度为输入**的：误差 1° 就输出 1 rad/s 的速度指令，
//   等效增益 57 rad/rad，非常硬。而 AS5600 只有 12bit（0.088°/LSB），
//   再加机械间隙和齿槽，到点后误差会在零点几个度内来回跳，
//   这点噪声被 57 倍放大 → 速度环持续出力 → 表现为"到位后原地嗡嗡震"。
//   死区内直接不出速度指令，让电机停住。
//   代价：稳态有 ±该值 的静差。0.2° ≈ 3.5 mrad，云台上肉眼不可见。
constexpr float POS_DEADBAND_DEG = 0.2f;

// ★ 位置环 P（输入单位是"度"）
//   串级关系：位置误差(度) × K → 速度指令(rad/s) → 速度环(P=.02,I=1) → 电流。
//   等效刚度 = K × 57.3 (rad/s per rad)。DengFOC 原值 1.0（= 57 rad/rad）是给
//   "空载小惯量电机 + 5kHz loop" 用的；装了云台后 M0 带着整个悬臂，惯量大十几倍，
//   同一个增益必然过冲 → 过冲后反向 → 极限环（实测在 ±11° 之间 14 rad/s 来回摆）。
//   0.2 = 等效 11.5 rad/s per rad，先用这个起步，用 'k' 命令现场试。
constexpr float POS_KP_DEFAULT = 0.2f;

// ★ 位置环速度阻尼 Kd（无量纲，辅助手段）
//   vel_cmd = Kp·误差(度) − Kd·实测速度(rad/s)
//   ★ 实测结论（2026-11-12）：单靠它压不住 M0 的摆 —— 因为它乘在速度环 P 上，
//     而 P 原来只有 0.02。真正的阻尼来自 VEL_KP_BASE（见上）。
//     现在 Kd 只做"死区内纯制动"：位置误差进死区后 vel_cmd=0，
//     Kd 让 `vel_cmd = −Kd·ω`，等于一个电子刹车，帮助快速收住残余摆动。
constexpr float POS_KD_DEFAULT = 0.3f;

constexpr float TRAJ_ACCEL     = 30.0f;   // 轨迹峰值加速度 rad/s²
// ★ S 曲线加加速度（jerk）上限 rad/s³
//   0 或负数 = 退回旧的梯形规划（加速度阶跃、启停有冲击）。
//   经验取「加速度的 2~4 倍」：太大就退化成梯形，太小则加速拖沓、动作变慢。
//   A=30、J=120 时，加速度 0→30 需要 0.25s，云台启停是"柔起柔停"而不是"一顿"。
//   实测代价：30° 动作从 0.24s（梯形）变 0.52s，180° 从 0.77s 变 1.04s。
//   嫌慢就把 J 加大（600 基本等于梯形），嫌启停有冲击就减小。
constexpr float TRAJ_JERK      = 120.0f;
constexpr int   QUEUE_SIZE     = 64;      // 每台电机指令队列深度

// ★ 位置命令（p）的隐式平滑
//   1 = 收到 p 命令后内部规划一条 S 曲线，位置环追中间点（默认，推荐）
//   0 = 老行为：目标直接砸给位置环 → 大误差 → vel_cmd 顶到限幅 → 猛冲
//   目标跳变时位置环 P=1 的输出会瞬间打到 VEL_LIMIT(30 rad/s)，这就是
//   "p 命令总感觉电机一顿、末端还过冲"的根源，不是 PID 参数问题。
constexpr int   POS_SMOOTH_ENABLE = 1;
constexpr float POS_SMOOTH_SPEED  = 5.0f; // 位置命令的默认行进速度 rad/s

constexpr float ALIGN_UQ       = 6.0f;    // 对齐时开环电压 V
constexpr int   ALIGN_RETRY    = 3;       // 对齐重试次数
constexpr int   ALIGN_SAMPLES  = 8;       // 每次对齐采样点数
constexpr float ALIGN_SETTLE   = 0.02f;   // 静止判据（采样离散度 rad）

constexpr int   SNAP_FRAMES    = 20;      // 故障快照帧数（20 × 100ms = 2 秒）
constexpr int   SNAP_MS        = 100;     // 快照帧间隔 ms

// 位置软限位默认值：云台 0° ~ 180°（0 ~ π rad）
// ★ 2026-11-12 改：原来是 ±180°。云台是"0~180° 往复摆动"的用法，
//   默认给 [0,180] 才符合直觉（p0 顶到下限、p180 顶到上限），
//   也不会往负方向转出去。装配标定后仍可用 'l' 命令改。
//   只用来防「多圈累积绕线 / 撞结构」这一件事。
constexpr float POS_LIMIT_LO_DEFAULT = 0.0f;
constexpr float POS_LIMIT_HI_DEFAULT = 3.14159265f;   // 180°

}  // namespace foc_param

// ============================================================
// 一台电机的硬件 / 电气配置
// 把原来散落在 #define 里的引脚、极对数、方向、母线电压全部收进配置结构体，
// 这样"加第三台电机"只是多填一个 MotorConfig，不用改控制代码。
// ============================================================
struct MotorConfig {
  int      index     = 0;        // 电机索引（pwm_* / protection_* / config_* 按索引调用）
  int      pin_a     = -1;
  int      pin_b     = -1;
  int      pin_c     = -1;
  int      pin_en    = -1;
  int      sda       = -1;
  int      scl       = -1;
  int      adc_a     = -1;
  int      adc_b     = -1;
  TwoWire* wire      = nullptr;  // M0 = &Wire, M1 = &Wire1（避免两颗 AS5600 抢地址）

  int   pole_pairs = 7;          // 极对数
  int   dir        = 1;          // 编码器方向（+1 / -1）
  float vbus       = 12.6f;      // 母线电压 V

  float iq_limit   = foc_param::IQ_LIMIT;    // 电流目标限幅 A
  float vel_limit  = foc_param::VEL_LIMIT;   // 位置环速度限幅 rad/s

  // ★ 速度环 P —— 必须**按轴**给，不能一刀切。
  //   同一个 P 对大惯量轴是"刚够阻尼"，对小惯量轴就是"过刚 + 放大噪声"：
  //     yaw (M0)：带整个悬臂，J≈1e-3 → 需要 P≈0.15 才不摆
  //     pitch(M1)：只有笔管，J≈2e-5 → P=0.15 会把测速噪声放大成高频细震，
  //                仿真里它自己就进了极限环；0.05 安静且 0.48s 收敛
  //   现场用 'w' 命令还能继续微调，这个是上电默认值。
  float vel_kp     = foc_param::VEL_KP_BASE;

  // ★ 对外坐标符号（只影响「用户看到的/发出去的」角度，不碰编码器与换相）
  //   背景：M1 编码器装反了 —— 内部累积角增大时，电机物理上是顺时针转，
  //   而用户要的坐标系是"角度增大 = 逆时针"。以前只能发负角度凑（1p-90 才是 +90°）。
  //   这里把符号放在**最外层**：
  //     · setTargetPos / setTargetVel / push 的入参先乘 user_sign
  //     · posLimitLo/Hi 输出时再乘回
  //     · 显示（STAT / 波形）乘回
  //   内部三环、换相、限位判断全部仍用内部坐标 → 数学上不可能引起正反馈，
  //   也绝不动 _cfg.dir 和 zero_electric_angle（那是上次踩爆的坑）。
  //   M0 = +1（正常），M1 = -1（装反）。
  float user_sign  = 1.0f;
};

// ============================================================
// 遥测一帧（供 foc_control_print 拼串口行）
// 语义固定不随模式变，target 的单位由 mode 决定：
//   MODE_CURRENT  → A       MODE_VELOCITY → rad/s
//   MODE_POSITION → rad     MODE_TRAJECTORY → rad
// ============================================================
struct MotorTelem {
  int   mode;
  float target;
  float position;   // 累积角度 rad
  float velocity;   // rad/s
  float iq;         // A
  float iq_cmd;     // A
  float vel_cmd;    // rad/s
  int   fault;
};

// ============================================================
// FOC_Motor —— 一台完整的 FOC 电机
//
// 把原来"Motor 结构体 + 18 个 foc_* 自由函数"收成一个类：
//   · begin(cfg)      ：引脚 + PWM + 编码器 + 电流采样 + 三环 PID + 使能 + NVS + 对齐
//   · update()        ：每 loop 跑一遍三环 + 保护 + 诊断（原 runControl）
//   · setTargetXxx()  ：设置目标（原 foc_set_target_*）
//   · push/queueXxx() ：轨迹队列（原 foc_queue_*）
// 双电机 = FOC_Motor g_motors[2]，每台自带全部状态，互不干扰。
// ============================================================
class FOC_Motor {
public:
  // ---------------- 生命周期（分两阶段，保持原初始化顺序）----------------
  // 阶段 1：引脚 + PWM + 编码器 + 电流零偏校准 + 三环 PID + 读 NVS
  //         此时【不使能、不对齐】——两台都 begin 完再 start，
  //         保证电流零偏是在"两颗电机都静止且不使能"下校准的（与原逻辑一致）
  void begin(const MotorConfig& cfg);

  // 阶段 2：使能 + 加载标定 + （按需）对齐
  // 返回 false = 对齐失败（沿用 NVS 里旧的零电角度）
  bool start();

  void update();                        // 每 loop 调用一次（三环 + 保护 + 诊断）

  bool align();                         // 开环拉转子测零电角度（存 NVS）
  void recalibrate();                   // 停转 → 重新对齐 → 存 NVS

  // ---------------- 模式 / 目标 ----------------
  void setMode(int m);
  int  mode() const { return _mode; }

  void  setTargetIq(float a);
  void  setTargetVel(float rad_s);
  void  setTargetPos(float rad);

  // 位置平滑开关（默认开）。
  //   开：setTargetPos 会规划一条 S 曲线，位置环追"移动中的中间点"——串口 p 命令就是这条。
  //   关：setTargetPos 直接把目标砸给位置环——CANopen 的 CSP（周期同步位置）要用这条，
  //       因为主站每 20ms 就换一个新目标，每段都重新规划会让速度被反复清零，跟不上。
  void  setSmooth(bool en) { _smooth_en = en; }
  bool  smoothEnabled() const { return _smooth_en; }

  float targetIq() const  { return _target_iq; }
  float targetVel() const { return _target_vel; }
  float targetPos() const { return _target_pos; }

  // ---------------- 反馈 ----------------
  float fullAngle() const { return _g_full; }
  // 累积角归一到软限位这段弧上（编码器漏计整圈时会有 ±360° 偏移）。
  // 对外显示/逆解都应该用这个，直接用 fullAngle 会拿到外圈值。
  float fullAngleNear() { return _nearFull(_g_full); }
  float velocity() const  { return _g_velocity; }
  float iq() const        { return _g_iq; }
  float velCmd() const    { return _vel_cmd; }

  // ---------------- 轨迹队列 ----------------
  bool push(float pos, float speed);    // 入队一条：目标机械角(rad) + 速度(rad/s)
  // 带段间停顿：该段到位后原地保持 dwell_ms 毫秒再执行下一条（0 = 不停）
  bool push(float pos, float speed, float dwell_ms);
  // 入队一条"归一化同步"命令：两轴用同一个 vnorm 就完全同步（云台指向用）
  bool pushNorm(float from, float dist, float span, float vmax);
  int  queueCount() const { return _q_count; }
  void queueStart();                    // 清空后开始执行（切轨迹模式，从当前位置起步）
  void queueStop();                     // 清空并立即停车（回速度模式 0）

  // ---------------- 标定 / NVS ----------------
  bool alignAlways() const { return _align_always; }
  void toggleAlignMode();
  void clearNvs() const;

  // ---------------- 位置软限位 ----------------
  // 限制的是「累积角度 _g_full」的绝对范围，防多圈绕线 / 防撞结构。
  // 默认 ±180°；装配标定后用串口 L 命令改小并存入 NVS，掉电不丢。
  // 目标超范围时钳位并打 [LIMIT] 警告（500ms 限流），不静默。
  void  setPosLimit(float lo_rad, float hi_rad);   // 设限 + 写 NVS

  // 在线调位置环 P（输入单位是"度"，1° 误差 → K rad/s 速度指令）。
  // 等效增益 = K × 57.3 (rad/rad)。原值 1.0 相当于 57 rad/rad，对带云台的
  // 大惯量关节太硬 —— 3° 误差就给 3 rad/s，冲过去刹不住 → 原地大幅摆。
  void  setPosKp(float k);
  float getPosKp() const { return _pos_kp; }

  // ★ 位置环速度阻尼（= 电子版的"用手扶一下"）
  //   vel_cmd = Kp·误差 − Kd·实测速度
  //   纯 P 位置环是**无阻尼二阶系统**：理论上是等幅振荡（临界稳定），
  //   实测表现就是"手扶能停、一松手/一轻拨又开始摆"。
  //   加这一项等于给系统补阻尼比，让"轻拨一下 → 回中 → 停住"变成收敛过程。
  //   Kd 无量纲：Kd=1 表示速度每多 1 rad/s 就从速度指令里减掉 1 rad/s。
  //   用**实测速度**做阻尼而不是误差微分，噪声小得多（12bit 编码器差分很脏）。
  void  setPosKd(float d);
  float getPosKd() const { return _pos_kd; }

  // ★★ 速度环 P —— 真正的阻尼旋钮
  //   串级里"阻尼比"是由**内环 P** 决定的，外环 Kd 只能间接、微弱地影响它。
  //   M0（竖直 yaw，无重力矩、轴承很滑）靠这一项才停得住。
  //   在线用 'w' 命令调：'w' 查看、'w0.15' 设置（允许 0.02 ~ 1.0）。
  void  setVelKp(float k);
  float getVelKp() const { return _vel_kp; }

  // 速度环输出（= iq 目标）限幅 A。这就是这台电机的**最大力矩上限**。
  //   位置环 P 再大，iq 也只能顶到这个值；所以"顶不住重力慢慢漂"往往是它太小，
  //   而不是 P 太小。在线可调就是为了现场分辨这两种情况。
  void  setVelIqLimit(float a);
  float getVelIqLimit() const { return _vel_iq_limit; }
  // 返回的是**对外坐标**（已按 user_sign 换算），串口 l 命令直接显示/使用
  float posLimitLo() const { return (_cfg.user_sign > 0.0f) ? _pos_lo : -_pos_hi; }
  float posLimitHi() const { return (_cfg.user_sign > 0.0f) ? _pos_hi : -_pos_lo; }
  bool  posLimitHit() const { return _limit_hit; }  // 上一次设目标是否被钳位

  // ★ 装配标定：把"此刻的位置"定为云台 0°
  //   1) 把当前绝对机械角写入编码器偏置 → 之后 full=0 就是这里
  //   2) 写 NVS，下次上电零点不变（不再"上电在哪就哪是 0"）
  //   3) 顺手把软限位重置成 [0, 180°]，并把目标/轨迹/积分器全部归零，
  //      避免标定瞬间因为 full 跳变而让电机猛冲一下。
  void calibrateZero();
  // 把"编码器绝对角 = abs_mech"这一刻定为云台 0°（零点误操作后的精确恢复）
  // reset_limit=false 时不动软限位，避免顺手把用户调好的限位冲掉
  void calibrateZeroAt(float abs_mech, bool reset_limit);

  // 当前零点在编码器坐标系里的绝对角（调试/显示用）
  float zeroOffset() const { return _zero_abs; }

  // ---------------- 遥测 ----------------
  // measure = true：用未滤波原始值 + 一阶平滑（量化测试用）
  MotorTelem telemetry(bool measure);

  const MotorConfig& config() const { return _cfg; }

  // 对外坐标符号（见 MotorConfig::user_sign）。显示用：值 = sign × 内部值
  float userSign() const { return _cfg.user_sign; }

  // 故障恢复时由外部通知（保护模块判定故障后调用 update 会自动处理）
  bool inFault() const { return _prev_fault; }

private:
  struct MotionCmd {
    float pos;    // 目标机械角度 rad
    float speed;  // 速度 rad/s（普通命令）/ 归一化速度 1/s（norm 命令）
    // ---- 归一化同步命令（双轴云台指向用）----
    // 普通命令：每轴各自规划 0→dist 的 S 曲线，形状随速度变化 → 两轴中途不同步。
    // norm 命令：两轴都按"本组最大行程 span"规划同一条曲线，再按 dist 缩放，
    //            同一时刻完成度完全相同（时间重标定），且小轴峰值自动等比缩小不超限。
    bool  norm  = false;
    float from  = 0.0f;   // 起点绝对角 rad
    float dist  = 0.0f;   // 有符号行程 rad
    float span  = 1.0f;   // 共用标尺 = 本组里最大的 |dist|（rad）
    // ★ 段间停顿（毫秒）：这一段跑完后在原地保持多久再执行下一条。
    //   0 = 不停顿（原来"段与段不减速、连续跑"的行为不变）。
    //   用途：模拟激光笔"抓到目标 → 停住 2 秒 → 跳到下一个目标"的跟踪节奏。
    float dwell = 0.0f;
  };
  struct SnapFrame {
    float pos;   // 累积角
    float vel;   // 速度反馈
    float iq;    // 实测 q 轴电流
    float vcmd;  // 位置环发出的速度指令
    float err;   // 位置误差
  };

  // ---------------- 内部实现 ----------------
  float _iqLowPass(float x);                       // 原 iqLowPass
  float _velLowPass(float x);                      // 原 velLowPass
  float _positionVelCmd(float pos_err_rad);        // 原 positionVelCmd
  float _runPositionMode(float dt, float full);              // 位置环，返回本拍位置误差
  float _runTrajectoryMode(float dt, float mech, float full); // 轨迹，返回本拍位置误差
  void  _printSnapOnFaultEntry(int fault);        // 进故障瞬间打印 2 秒快照
  void  _updateStuckDiag(float pos_err, float velocity, float iq, float iq_cmd);
  void  _pushSnap(float pos, float vel, float iq, float vcmd, float err);
  void  _updateProtection(float iq, float velocity, float pos_err, float full);
  float _clampTarget(float v);                     // 软限位钳位（超限打警告，500ms 限流）

  // ★ 把累积角折算到"离限位区间最近的等价角"（±2π 整数倍）
  //   用途：防止编码器漏计/多计一圈后 full 落在 360° 之外，
  //   让位置误差突然变成几百度量级 → 电机被逼着绕一整圈。
  //   正常情况（full 在 [lo,hi] 内）返回值等于 full 本身。
  float _nearFull(float full);

  // ---------------- 配置 ----------------
  MotorConfig _cfg;

  // ---------------- 硬件对象 ----------------
  FOC_Encoder _encoder;
  FOC_Current _current;
  FOC_PID     _current_pid;
  FOC_PID     _velocity_pid;
  FOC_PID     _position_pid;

  // ---------------- 标定 ----------------
  bool  _align_always      = true;
  float _zero_electric_angle = 0.0f;

  // ---------------- 位置软限位 ----------------
  float    _pos_lo        = foc_param::POS_LIMIT_LO_DEFAULT;
  float    _pos_hi        = foc_param::POS_LIMIT_HI_DEFAULT;
  float    _zero_abs      = 0.0f;   // 云台 0° 对应的编码器绝对角（标定值）
  uint32_t _limit_warn_ts = 0;
  bool     _limit_hit     = false;

  // ---------------- 目标 + 模式 ----------------
  float _target_iq  = 0.0f;
  float _target_vel = 0.0f;
  float _target_pos = 0.0f;
  int   _mode       = MODE_VELOCITY;
  bool  _smooth_en  = true;    // 位置目标是否走 S 曲线（CANopen CSP 模式下关掉）

  // ---------------- 位置命令的隐式 S 曲线 ----------------
  // 收到 p 命令时不直接把目标砸给位置环，而是先规划一条 S 曲线，
  // 位置环追"移动中的中间点"。用户指令不变，但启停从"猛冲"变"柔推"。
  bool  _pos_smooth = false;   // true = 正在按 S 曲线逼近（_traj 正在跑）
  float _pos_speed  = foc_param::POS_SMOOTH_SPEED;
  float _pos_kp     = foc_param::POS_KP_DEFAULT;   // 位置环 P（度为单位）
  float _pos_kd     = foc_param::POS_KD_DEFAULT;   // 位置环速度阻尼（无量纲）
  float _vel_kp     = foc_param::VEL_KP_BASE;      // 速度环 P（真正的阻尼旋钮）
  float _vel_iq_limit = foc_param::VEL_LIMIT_NEAR; // 速度环输出限幅 A（力矩上限）

  // ---------------- 滤波状态 ----------------
  float    _iq_filt_y  = 0.0f;
  uint32_t _iq_filt_ts = 0;
  float    _vel_filt_y = 0.0f;
  uint32_t _vel_filt_ts = 0;

  // ---------------- 遥测缓存 ----------------
  float _g_full     = 0.0f;
  float _g_velocity = 0.0f;
  float _g_iq       = 0.0f;
  float _g_iq_cmd   = 0.0f;
  float _g_vel_raw  = 0.0f;
  float _g_iq_raw   = 0.0f;
  float _vel_cmd    = 0.0f;   // 位置环输出的速度指令
  float _mv         = 0.0f;   // 测量模式平滑缓存（速度）
  float _mi         = 0.0f;   // 测量模式平滑缓存（电流）

  // ---------------- 卡住监测 ----------------
  uint32_t _stuck_since_ms = 0;
  bool     _stuck_reported = false;

  // ---------------- 故障恢复 ----------------
  bool _prev_fault = false;
  uint32_t _fault_log_ts = 0;    // 故障中"还在故障里"的重复提示计时（3s 一次）

  // 上电自检是否通过（编码器正常 + 零位可信）。
  // false = 这台电机不参与控制，update() 直接返回、输出 coast。
  // ★ 存在的理由：原来 align() 失败后 start() 只是 return false，
  //   但 pwm_enable() 早已调用、电机仍然使能，而 _zero_electric_angle 是旧值/0 →
  //   用错的电气角做 Park 变换 = 换相错乱 → 电机"大范围乱抽"。
  //   这是概率性的（取决于对齐时转子有没有被重力拖走），所以之前很难复现。
  bool _boot_ok = true;

  // ---------------- 轨迹 ----------------
  TrajState _traj          = {};
  uint32_t  _loop_us       = 0;      // 上一拍时间戳（算真实 dt）
  float     _last_cmd_pos  = 0.0f;   // 上一条指令目标（180° 步进定方向）
  bool      _has_cmd       = false;

  // ---------------- 指令队列（环形）----------------
  MotionCmd _queue[foc_param::QUEUE_SIZE] = {};
  int       _q_head  = 0;
  int       _q_count = 0;
  int       _q_total = 0;              // 本轮入队总条数（打印 #n/总数）
  // ---- 当前正在执行的归一化命令（用于把 0..1 进度还原成绝对角）----
  bool      _sync_norm = false;
  float     _sync_from = 0.0f;
  float     _sync_dist = 0.0f;
  float     _sync_span = 1.0f;
  bool      _q_done_reported = false;  // "队列跑完"提示只打一次

  // ---- 段间停顿（dwell）----
  //   _cur_dwell_ms ：当前这段要求的停顿毫秒数（出队时记录）
  //   _dwell_until  ：停顿截止时刻（millis）。0 = 不在停顿中
  //   语义：轨迹跑完 → 位置环锁在终点 → 到点等够时间 → 再出队下一条。
  float     _cur_dwell_ms = 0.0f;
  uint32_t  _dwell_until  = 0;

  // ---------------- 故障现场快照 ----------------
  SnapFrame _snap[foc_param::SNAP_FRAMES] = {};
  int       _snap_i  = 0;
  uint32_t  _snap_ts = 0;
};

#endif  // FOC_MOTOR_H
