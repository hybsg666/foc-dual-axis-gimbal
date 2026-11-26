#include "foc_serial.h"
#include <Arduino.h>
#include <ctype.h>
#include "foc_pwm.h"        // MOTOR_0 / NUM_MOTORS
#include "foc_control.h"
#include "foc_gimbal.h"
#include "foc_protection.h"
#include "foc_config.h"
#include "canopen.h"        // t 命令：CANopen 板内回环自检

// ------------------------------------------------------------
// 协议（命令前可加 0/1 前缀选择电机，不带前缀 = 电机 0）：
//   "i0.3"      → 电机0 电流模式，目标 0.3A
//   "v10"       → 电机0 速度模式，目标 10 rad/s
//   "1v10"      → 电机1 速度模式
//   "p3.14"     → 电机0 位置模式
//   "1p3.14"    → 电机1 位置模式
//   "r" "0r" "1r"  → 清故障（可指定电机）
//   "c" "0c" "1c"  → 重标定零电角度并保存
//   "x" "0x" "1x"  → 清空该电机 NVS
//   "s" "0s" "1s"  → 查看该电机 NVS 存档 + 对齐模式
//   "a" "0a" "1a"  → 切换该电机对齐模式
//   "l" "0l" "1l"  → 查看该电机软限位
//   "l170"  "1l60"  → 设对称限位 ±170°（单位：度）
//   "l-170,170"     → 设非对称限位（单位：度）
//   "m"        → 切换测量模式（全局）
//   "?"        → 状态总览：两台的模式/故障码/目标/实际/误差/软限位
//   "g x,y,z"  → ★指向命令：给空间方向，自动逆解 yaw/pitch 并双轴同步到位
//                "g0,1,0" 正前方 / "g1,0,0" 右方 / "g0,1,0.5" 前方抬高
//                "g x,y,z,v" 可指定速度上限
//   "z" "z30,10" → 查看/设置云台零位偏置（度，装配标定用）
//   "y" "y1"     → 翻转/设置 yaw 关节方向符号（指向反了就发 y）
//   "q0,1.57,2;1,0.5,1;0,-1.57,2" → 轨迹队列：每条=电机号,目标位置(rad),速度(rad/s)，分号隔开
//   "e" "0e" "1e"  → 清空该电机指令队列并停止
//   "0.5"      → 兼容旧行为：电机0 速度模式 0.5
// ------------------------------------------------------------

// 单字母命令（**永远不带参数**）：收到就能立即执行，不必等换行符
//
// ★ 只放"没有参数形式"的命令。带参数的必须等 '\n' 或长超时。
//   踩过的坑：'0k0.05' 里的 "0k" 命中了这张表 → 立即 flush 执行"查看"，
//   剩下的 "0.05" 单独成为一条命令，落到兜底被当成速度 0.05 rad/s →
//   电机以 0.05 rad/s 匀速转，表现成"发个调参命令，电机就自己慢慢转、还变软"。
//   同理 'l' / 'z' / 'y' 也都能带参数，一并移出。
static bool isSingleLetterCmd(char k) {
  k = (char)tolower((unsigned char)k);
  return k == 'r' || k == 'c' || k == 'x' || k == 's' || k == 'a' ||
         k == 'm' || k == 'e' || k == 'd' || k == 'h' || k == '?' || k == 't';
}

// 判断当前缓冲是不是已经是一条完整的单字母命令（允许 "c"、"0c"、"1a"）
static bool isCompleteSingleCmd(const String& b) {
  int start = 0;
  if (b.length() >= 2 && (b[0] == '0' || b[0] == '1') && isalpha((unsigned char)b[1])) {
    start = 1;
  }
  return (b.length() - start == 1) && isSingleLetterCmd(b.charAt(start));
}

// 解析电机前缀 + 命令字 + 参数
// "0v5"    -> motor=0, k='v', arg="5"
// "1p3.14" -> motor=1, k='p', arg="3.14"
// "v5"     -> motor=0, k='v', arg="5"
// "0.5"    -> motor=0, k='0', arg=".5"（走 else 分支，速度模式）
static void splitCmd(const String& b, int& motor, char& k, String& arg) {
  motor = MOTOR_0;
  int start = 0;
  if (b.length() >= 2 && (b[0] == '0' || b[0] == '1') && isalpha((unsigned char)b[1])) {
    motor = b[0] - '0';
    start = 1;
  }
  k = b.charAt(start);
  arg = b.substring(start + 1);
}

void foc_serial_update() {
  static String   buf;
  static uint32_t last_rx = 0;

  // 三种方式都算"一条命令结束"：\n\r / 单字母命令立即执行 / 800ms 超时
  bool flush = false;
  while (Serial.available()) {
    char c = (char)Serial.read();
    last_rx = millis();
    if (c == '\n' || c == '\r') { flush = true; break; }
    buf += c;
    if (isCompleteSingleCmd(buf)) { flush = true; break; }
  }
  // q 命令带超长参数，必须等换行符，不启用 200ms 超时（否则长指令会被截断）
  bool isQueue = (buf.length() > 0 && (buf[0] == 'q' || buf[0] == 'Q'));
  if (!flush && buf.length() > 0 && !isQueue && (millis() - last_rx) > 800) flush = true;
  if (!flush) return;

  buf.trim();
  last_rx = millis();
  if (buf.length() == 0) return;

  int   motor;
  char  k;
  String arg;
  splitCmd(buf, motor, k, arg);

  // ★ 故障中收到"要动"的命令 → 先自动清故障再执行。
  //   以前故障时电机静默罢工、串口无提示，表现就是"发什么命令都不动，像死机"。
  //   调试期这样最省事：只要你还在发运动命令，就当你要继续干活。
  //   （r 命令本身、查询类命令不过这里，不受影响）
  if (foc_protection_is_fault(motor) &&
      (k == 'p' || k == 'P' || k == 'v' || k == 'V' ||
       k == 'i' || k == 'I' || k == 'q' || k == 'Q')) {
    Serial.printf("[main] m%d was in FAULT 0x%02X -> auto clear, executing\n",
                  motor, foc_protection_get_fault(motor));
    foc_protection_clear(motor);
  }

  // 该电机收到指令，刷新指令超时计时
  foc_protection_notify_cmd(motor);

  if (k == '?') {
    // 状态总览：卡住时先发这条，一眼看出"是故障 / 是限位 / 还是没收到命令"
    static const char* MN[4] = {"CURRENT", "VELOCITY", "POSITION", "TRAJECTORY"};
    const float R2D = 57.2958f;
    for (int mm = 0; mm < NUM_MOTORS; mm++) {
      int   md  = foc_get_mode(mm);
      int   fc  = foc_protection_get_fault(mm);
      const float s  = foc_get_user_sign(mm);   // 对外坐标符号（M1 = -1）
      float tgt = s * foc_get_target_pos(mm);
      float now = s * foc_control_get_full(mm);
      float vel = s * foc_control_get_velocity(mm);
      float iq  = foc_control_get_iq(mm);
      float jl  = foc_get_vel_iq_limit(mm);
      // 饱和标记：|iq| 贴住限幅 = 力矩已经用尽（想顶也顶不住了）
      bool sat = (fabsf(iq) > jl * 0.9f);
      Serial.printf("[STAT] m%d mode=%s fault=0x%02X target=%.1fdeg now=%.1fdeg err=%+.1fdeg "
                    "vel=%.2f iq=%.2f/%.2f%s kp=%.2f kd=%.2f vkp=%.3f limit=[%.1f,%.1f]deg%s\n",
                    mm, (md >= 0 && md < 4) ? MN[md] : "?", fc,
                    (double)(tgt * R2D), (double)(now * R2D), (double)((tgt - now) * R2D),
                    (double)vel, (double)iq, (double)jl, sat ? " SAT" : "",
                    (double)foc_get_pos_kp(mm), (double)foc_get_pos_kd(mm),
                    (double)foc_get_vel_kp(mm),
                    (double)(foc_get_pos_limit_lo(mm) * R2D),
                    (double)(foc_get_pos_limit_hi(mm) * R2D),
                    foc_get_pos_limit_hit(mm) ? " CLAMPED" : "");
      if (fc) Serial.printf("[STAT]   ^^ m%d IS IN FAULT, send '%dr' to clear\n", mm, mm);
    }
    Serial.printf("[STAT] ctrl loop = %.0f Hz\n", (double)foc_control_get_hz());
    return;

  } else if (k == 'i' || k == 'I') {
    foc_set_mode(motor, MODE_CURRENT);
    foc_set_target_iq(motor, arg.toFloat());
    Serial.printf("[main] m%d mode=CURRENT, target_iq=%.3f\n", motor, foc_get_target_iq(motor));

  } else if (k == 'p' || k == 'P') {
    // ★ 2026-11-12：p 的参数单位从 rad 改成 **度**
    //   用户要的是"0p0 = 0°、0p180 = 180°"，rad（3.1416）不适合现场用。
    //   'p90' = 90°，'1p45' = 电机1 的 45°。
    const float D2R = 0.01745329252f;
    const float R2D = 57.2957795f;
    float deg = arg.toFloat();
    float rad = deg * D2R;
    foc_set_mode(motor, MODE_POSITION);
    foc_set_target_pos(motor, rad);
    // 目标 / 当前 / 误差全部用度打印，一眼看出差多少、方向对不对
    {
      const float s = foc_get_user_sign(motor);
      float tgt = s * foc_get_target_pos(motor);
      float now = s * foc_control_get_full(motor);
      Serial.printf("[main] m%d mode=POSITION, target=%.1f deg, now=%.1f deg, err=%+.1f deg\n",
                    motor, (double)(tgt * R2D), (double)(now * R2D),
                    (double)((tgt - now) * R2D));
    }

  } else if (k == 'v' || k == 'V') {
    foc_set_mode(motor, MODE_VELOCITY);
    foc_set_target_vel(motor, arg.toFloat());
    Serial.printf("[main] m%d mode=VELOCITY, target_vel=%.3f\n", motor,
                  (double)(foc_get_user_sign(motor) * foc_get_target_vel(motor)));

  } else if (k == 'r' || k == 'R') {
    foc_protection_clear(motor);

  } else if (k == 'c' || k == 'C') {
    foc_control_recalibrate(motor);

  } else if (k == 'o' || k == 'O') {
    // 手动当主站发一帧 SDO：o40 00 10 00（空格分隔的十六进制字节）
    String hex = arg;
    hex.trim();
    co_manual_sdo(hex.c_str());

  } else if (k == 't' || k == 'T') {
    // CANopen 板内回环自检：不接主站也能验证协议栈（阻塞约 0.3 秒）
    co_self_test();

  } else if (k == 'x' || k == 'X') {
    foc_control_clear_nvs(motor);

  } else if (k == 's' || k == 'S') {
    float zv = foc_config_load_zero_angle(motor);
    if (isnan(zv)) {
      Serial.printf("[main] m%d NVS: no zero_angle stored\n", motor);
    } else {
      Serial.printf("[main] m%d NVS stored zero_angle = %.4f\n", motor, zv);
    }
    Serial.printf("[main] m%d align_mode = %s\n", motor,
                  foc_control_get_align_mode(motor)
                    ? "ALWAYS (dev: 每次上电对齐)"
                    : "STORED (prod: 有存档则跳过对齐)");

  } else if (k == 'j' || k == 'J') {
    // 速度环输出限幅 = 该轴最大力矩上限（A）
    //   'j'    查看    'j1.2' 设当前电机    '1j1.2' 设电机1
    String s = arg;
    s.trim();
    if (s.length() == 0) {
      Serial.printf("[main] m%d vel iq_limit = %.2f A (力矩上限)\n",
                    motor, (double)foc_get_vel_iq_limit(motor));
      Serial.printf("[main]   用法：'j1.2' 放开到 1.2A，'j0.5' 收回；上限 2.5A\n");
    } else {
      foc_set_vel_iq_limit(motor, s.toFloat());
    }

  } else if (k == 'k' || k == 'K') {
    // 位置环：'k' 查看 / 'k0.1' 设 P / 'k0.1,0.3' 设 P 和 阻尼 / 'k,0.3' 只设阻尼
    String s = arg;
    s.trim();
    if (s.length() == 0) {
      Serial.printf("[main] m%d pos_kp = %.3f (等效 %.1f rad/s per rad), pos_kd = %.3f\n",
                    motor, (double)foc_get_pos_kp(motor),
                    (double)(foc_get_pos_kp(motor) * 57.2958f),
                    (double)foc_get_pos_kd(motor));
      Serial.printf("[main]   用法：'k0.1' 只调 P；'k0.1,0.3' 同时调 P 和阻尼；'k,0.5' 只调阻尼\n");
    } else {
      int c = s.indexOf(',');
      String sp = (c < 0) ? s : s.substring(0, c);
      String sd = (c < 0) ? String("") : s.substring(c + 1);
      sp.trim();
      sd.trim();
      if (sp.length() > 0) foc_set_pos_kp(motor, sp.toFloat());
      if (sd.length() > 0) foc_set_pos_kd(motor, sd.toFloat());
    }

  } else if (k == 'w' || k == 'W') {
    // ★ 速度环 P：串级系统里真正决定"摆不摆"的旋钮
    //   'w' 查看 / 'w0.15' 设置 / '0w0.25' 只设电机 0
    //   调这个而不是 k：位置环 Kd 最终只把阻尼系数从 Pv 抬到 Pv·(1+Kd)，
    //   Pv=0.02 时那点增量根本看不出来（已实测）。
    String s = arg;
    s.trim();
    if (s.length() == 0) {
      Serial.printf("[main] m%d vel_kp = %.3f (速度环 P)\n",
                    motor, (double)foc_get_vel_kp(motor));
      Serial.printf("[main]   用法：'w0.15' 默认；摆不停就往上加(0.25/0.35)，"
                    "出现细密嗡嗡或发烫就往下收；范围 0.02~1.0\n");
    } else {
      foc_set_vel_kp(motor, s.toFloat());
    }

  } else if (k == 'h' || k == 'H') {
    // ★ 装配标定：把"此刻的位置"定为云台 0°
    //   'h' 或 '0h'   -> 电机0；'1h' -> 电机1。会写 NVS，限位重置成 [0, 180°]
    //   'h5.0621'     -> 把"编码器绝对角 5.0621 rad"这一刻定为 0°（限位不动）
    //                    ↑ 零点被误改过之后，用这条精确恢复，不用把轴搬回去
    String s = arg;
    s.trim();
    if (s.length() == 0) {
      foc_calibrate_zero(motor);
    } else {
      // 允许写成 "h=5.0621" / "h,5.0621"，手滑打了分隔符也不至于报错
      if (s.charAt(0) == '=' || s.charAt(0) == ',') s = s.substring(1);
      float z = s.toFloat();
      if (z == 0.0f && s != "0" && s != "0.0") {
        Serial.printf("[main] 'h' 参数看不懂：'%s'（要么不写，要么写编码器弧度，如 h5.0621）\n",
                      s.c_str());
      } else {
        foc_calibrate_zero_at(motor, z);
      }
    }

  } else if (k == 'a' || k == 'A') {
    foc_control_toggle_align_mode(motor);

  } else if (k == 'l' || k == 'L') {
    // 软限位：不带参数 = 查看；带参数 = 设置（单位度，写入 NVS）
    String s = arg;
    s.trim();
    const float R2D = 57.2958f;
    const float D2R = 0.0174533f;

    if (s.length() == 0) {
      Serial.printf("[main] m%d pos_limit = [%.1f, %.1f] deg  (now %.1f deg)\n",
                    motor,
                    foc_get_pos_limit_lo(motor) * R2D,
                    foc_get_pos_limit_hi(motor) * R2D,
                    foc_get_user_sign(motor) * foc_control_get_full(motor) * R2D);
      Serial.printf("[main]   用法：'l180' = [0,180°]（默认）；'l160' = [0,160°]；'l-10,170' = 非对称\n");
    } else {
      int c = s.indexOf(',');
      if (c > 0) {
        float lo = s.substring(0, c).toFloat() * D2R;
        float hi = s.substring(c + 1).toFloat() * D2R;
        foc_set_pos_limit(motor, lo, hi);
      } else {
        // 单个数字 = 上限 x，下限固定 0（云台是 0~x 的用法）
        //   旧版是"对称 ±x"，但云台 0~180° 的场景下 [0,x] 才直观。
        //   要对称范围就直接写 'l-90,90'。
        float x = s.toFloat() * D2R;
        foc_set_pos_limit(motor, 0.0f, x);
      }
      Serial.printf("[main] m%d pos_limit now = [%.1f, %.1f] deg  (now %.1f deg)\n",
                    motor,
                    foc_get_pos_limit_lo(motor) * R2D,
                    foc_get_pos_limit_hi(motor) * R2D,
                    foc_get_user_sign(motor) * foc_control_get_full(motor) * R2D);
    }

  } else if (k == 'm' || k == 'M') {
    foc_control_set_measure(!foc_control_get_measure());

  } else if (k == 'd' || k == 'D') {
    // 诊断模式：关遥测、只留诊断行，方便抓偶发 bug
    foc_control_set_diag(!foc_control_get_diag());

  } else if (k == 'e' || k == 'E') {
    // 清空该电机队列 + 立即停止
    foc_queue_stop(motor);
    Serial.printf("[main] m%d queue cleared + stopped\n", motor);

  } else if (k == 'g' || k == 'G') {
    // 指向命令（阶段三核心）：g x,y,z[,v]
    //   直接给空间方向向量，内部逆解出 yaw/pitch 并让两轴同时到位
    //   例：g0,1,0 = 正前方；g1,0,0 = 右方；g0,1,1 = 前方抬高 45°
    String s = arg;
    s.trim();
    if (s.length() == 0) {
      float yd, pd;
      foc_gimbal_get_offset(&yd, &pd);
      Serial.printf("[GIMBAL] zero offset: yaw=%+.1f pitch=%+.1f deg\n", yd, pd);
      Serial.printf("[GIMBAL] usage: g x,y,z[,speed]   e.g. g0,1,0  g0.5,1,0.3\n");
    } else {
      int c1 = s.indexOf(',');
      int c2 = (c1 < 0) ? -1 : s.indexOf(',', c1 + 1);
      int c3 = (c2 < 0) ? -1 : s.indexOf(',', c2 + 1);
      if (c1 < 0 || c2 < 0) {
        Serial.printf("[GIMBAL] bad cmd, usage: g x,y,z[,speed]\n");
      } else {
        float gx = s.substring(0, c1).toFloat();
        float gy = s.substring(c1 + 1, c2).toFloat();
        float gz = (c3 < 0) ? 0.0f : s.substring(c2 + 1, c3).toFloat();
        float gv = (c3 < 0) ? 0.0f : s.substring(c3 + 1).toFloat();
        if (!foc_gimbal_point(gx, gy, gz, gv)) {
          Serial.printf("[GIMBAL] invalid direction: x/y/z all zero\n");
        }
      }
    }

  } else if (k == 'y' || k == 'Y') {
    // yaw 方向符号：y = 翻转，y1 / y-1 = 直接设置
    String s = arg;
    s.trim();
    if (s.length() == 0) {
      foc_gimbal_set_yaw_sign(-foc_gimbal_get_yaw_sign());
    } else {
      foc_gimbal_set_yaw_sign(s.toFloat());
    }

  } else if (k == 'z' || k == 'Z') {
    // 零位偏置：z = 查看，z yaw_deg,pitch_deg = 设置（装配后标定用）
    String s = arg;
    s.trim();
    if (s.length() == 0) {
      float yd, pd;
      foc_gimbal_get_offset(&yd, &pd);
      Serial.printf("[GIMBAL] zero offset: yaw=%+.1f pitch=%+.1f deg  (usage: z yaw,pitch)\n",
                    yd, pd);
    } else {
      int c = s.indexOf(',');
      float yd = (c > 0) ? s.substring(0, c).toFloat() : s.toFloat();
      float pd = (c > 0) ? s.substring(c + 1).toFloat() : 0.0f;
      foc_gimbal_set_offset(yd, pd);
    }

  } else if (k == 'q' || k == 'Q') {
    // 轨迹队列命令：q 电机号,位置,速度;电机号,位置,速度;...
    // 语义 = 清空两个电机队列 → 逐条入队 → 两个电机都切轨迹模式开始执行
    // q 的电机号写在参数里（不是前缀），所以两台都要检查/清故障
    for (int mm = 0; mm < NUM_MOTORS; mm++) {
      if (foc_protection_is_fault(mm)) {
        Serial.printf("[main] m%d was in FAULT 0x%02X -> auto clear (q)\n",
                      mm, foc_protection_get_fault(mm));
        foc_protection_clear(mm);
      }
    }
    for (int mm = 0; mm < NUM_MOTORS; mm++) foc_queue_stop(mm);

    String s = arg;
    s.trim();
    int parsed = 0;
    int idx = 0;
    while (idx < s.length()) {
      int sep = s.indexOf(';', idx);
      String one = (sep < 0) ? s.substring(idx) : s.substring(idx, sep);
      one.trim();
      if (one.length() > 0) {
        int c1 = one.indexOf(',');
        int c2 = (c1 < 0) ? -1 : one.indexOf(',', c1 + 1);
        int c3 = (c2 < 0) ? -1 : one.indexOf(',', c2 + 1);
        if (c1 > 0 && c2 > c1) {
          int   cm = one.substring(0, c1).toInt();
          // ★ 2026-11-12：位置参数和 p 命令统一成「度」，内部再转 rad。
          //   之前这里还是 rad，导致 q 和 p 两套单位，现场极易写错。
          float p  = one.substring(c1 + 1, c2).toFloat() * 0.01745329252f;
          // ★ 可选第 4 字段 = 该段到位后原地停顿的毫秒数
          //   例：q0,120,4,2000;1,60,3,2000 → 两轴到位后各自停 2 秒再走下一段
          String vs = (c3 > c2) ? one.substring(c2 + 1, c3) : one.substring(c2 + 1);
          float v  = vs.toFloat();
          float dw = (c3 > c2) ? one.substring(c3 + 1).toFloat() : 0.0f;
          if (cm >= 0 && cm < NUM_MOTORS && foc_queue_push(cm, p, v, dw)) parsed++;
        }
      }
      if (sep < 0) break;
      idx = sep + 1;
    }

    for (int mm = 0; mm < NUM_MOTORS; mm++) foc_queue_start(mm);
    Serial.printf("[main] queue: parsed %d -> m0=%d m1=%d (mode=TRAJECTORY, 位置单位=度)\n",
                  parsed, foc_queue_count(MOTOR_0), foc_queue_count(MOTOR_1));

  } else if (!isalpha((unsigned char)k)) {
    // ★ 纯数字残片（如命令被拆断后剩下的 "1.2"）→ 明确拒收，绝不执行。
    //   以前这里会当成速度命令：'0k0.5' 拆断后 "0.5" 就让电机以 0.5 rad/s 转。
    //   速度命令请一律写 'v0.5' / '1v0.5'。
    Serial.printf("[main] stray number '%s' -> IGNORED (速度命令请用 'v0.5')\n", buf.c_str());

  } else {
    // ★ 兜底：不认识的命令 → 明确拒绝，绝不"猜"。
    //   以前这里无条件当速度命令，于是 "0j1.8"（本意是调力矩上限）落到这里，
    //   被 buf.toFloat() 取成 1.8 → 电机突然以 1.8 rad/s 匀速转，
    //   表现成"发个参数命令，电机自己顺时针走"，非常难查。
    //   嵌入式里"把不认识的输入当有效指令"是最危险的一类 bug。
    Serial.printf("[main] unknown cmd '%s' (motor=%d key='%c') -> IGNORED\n",
                  buf.c_str(), motor, k);
  }

  buf = "";
}
