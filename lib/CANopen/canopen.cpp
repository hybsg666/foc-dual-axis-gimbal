#include "canopen.h"
#include "foc_can.h"
#include "foc_control.h"     // 0x6064 实际位置要从三环里取
#include <string.h>
#include <Preferences.h>     // 第 10 步：参数落盘（NVS），断电不丢

// ============================================================
// 最小 CANopen 从站：NMT + 心跳 + SDO（expedited 传输）
//
// 帧 ID 速查（Classic CAN，11 位 ID，500 kbps）：
//   0x000            NMT          主站 → 从站
//   0x700 + node     心跳         从站 → 主站（1 字节 = 状态）
//   0x600 + node     SDO 请求     主站 → 从站
//   0x580 + node     SDO 应答     从站 → 主站
// ============================================================

// ---------- 节点状态（CiA 301 规定值，不要改）----------
enum : uint8_t {
  CO_ST_BOOT        = 0x00,
  CO_ST_STOPPED     = 0x04,
  CO_ST_OPERATIONAL = 0x05,
  CO_ST_PREOP       = 0x7F,
};

// ---------- CiA 402 驱动器状态机（伺服专用，共 7 档）----------
//   上电后只能停在"Switch on disabled"，必须按顺序被主站推着走：
//     Switch on disabled --(写 0x0006)--> Ready to switch on
//                        --(写 0x0007)--> Switched on
//                        --(写 0x000F)--> Operation enabled   ← 这一档才允许出转矩
//                        --(写 0x0000)--> Switch on disabled   ← 断电压，回起点
enum : uint8_t {
  CIA_NOT_READY   = 0,
  CIA_SWITCH_DIS  = 1,
  CIA_READY       = 2,
  CIA_SWITCHED_ON = 3,
  CIA_OP_ENABLED  = 4,
  CIA_FAULT       = 5,
};

static const char* cia_state_name(uint8_t s) {
  switch (s) {
    case CIA_NOT_READY:   return "Not ready to switch on";
    case CIA_SWITCH_DIS:  return "Switch on disabled";
    case CIA_READY:       return "Ready to switch on";
    case CIA_SWITCHED_ON: return "Switched on";
    case CIA_OP_ENABLED:  return "Operation enabled";
    case CIA_FAULT:       return "Fault";
    default:              return "?";
  }
}

// 把"状态字"反解成状态名：主站读 0x6041 后就是这么判断的。
// ★ 掩码必须是 0x006F（bit0,1,2,3,5,6）：
//   bit5 "Quick stop" 为 1 表示"没有急停"，是 Ready/Switched/Enabled 的必要条件。
//   以前写成 0x004F 漏了 bit5 → 0x0231 算出来是 0x0001、0x0237 算成 0x0007，
//   于是显示了 "?" 和 "Quick stop active"。踩过。
static const char* cia_state_from_sw(uint16_t sw) {
  switch (sw & 0x006Fu) {
    case 0x0000: return "Not ready to switch on";
    case 0x0040: return "Switch on disabled";
    case 0x0021: return "Ready to switch on";
    case 0x0023: return "Switched on";
    case 0x0027: return "Operation enabled";
    case 0x0007: return "Quick stop active";
    case 0x000F: return "Fault reaction active";
    case 0x0008: return "Fault";
    default:     return "?";
  }
}

struct CoNode {
  uint8_t  node;        // 节点号 1 / 2
  uint8_t  state;       // 上面那四个状态之一
  uint32_t hb_period;   // 0x1017，心跳周期 ms（0 = 关闭）
  uint32_t hb_last;     // 上次发心跳的时刻
  uint8_t  cia;         // CiA 402 档位（cia_state_name 的那个）
  uint16_t control;     // 0x6040 最近一次写入的控制字，读它时原样返回
  int32_t  target;      // 0x607A 目标位置（0.01 度，用户坐标），到位判断用
  uint32_t pdo_period;  // TPDO 自动上报周期 ms（0 = 关闭）
  uint32_t pdo_last;    // 上次发 TPDO 的时刻

  // ---- 第 6 步：SYNC + 心跳超时 ----
  uint16_t pend_cw;     // SYNC 模式：缓存的控制字
  int32_t  pend_pos;    // SYNC 模式：缓存的目标位置
  bool     has_pend;    // 有缓存，等下一帧 SYNC 一起生效
  uint8_t  mon_node;    // 0x1016：要盯着谁的心跳（主站节点号）
  uint32_t mon_ms;      // 0x1016：多久没动静算掉线（0 = 不监控）
  uint32_t mon_last;    // 最近一次收到它心跳的时刻

  // ---- 第 8 步：故障 ----
  uint16_t err_code;    // 0x603F：上一次故障的错误码（0 = 没故障）
};

// 0x1006 通信周期（µs）。0 = 没有 SYNC，RPDO 收到就立刻生效（第 4 步的老行为）；
// 非 0 = SYNC 模式，RPDO 先缓存，等 0x080 到了两轴才一起动。
static uint32_t g_sync_us  = 0;
static uint8_t  g_sync_cnt = 0;

// 连续轨迹时每 20ms 换一个目标，那行"节点X 目标 ..."会 1 秒刷 50 行，
// 把真正要看的统计淹掉。这个开关只在轨迹函数里临时打开。
static bool g_tgt_quiet = false;

// ------------------------------------------------------------
// 限速打印（踩过的坑，2026-11-24）
// ------------------------------------------------------------
// 现象：F103 主站开始"画圆"（每 20ms 一帧 0x201 + 0x202）之后，
//       两轴立刻不动了，而且手一推是软的。
//
// 根因不是控制环、也不是限位，是**串口被自己刷死**：
//   每帧 3 行日志（收到 / 目标 / SMOOTH），两轴 50Hz →
//   约 18000 字节/秒，而 115200 波特率只能跑 11520 字节/秒。
//   Serial.printf 在发送缓冲满时会**阻塞**，主循环跟着停，
//   foc_control_update() 从 500Hz 掉到 100~200Hz →
//   PID 积分与采样率失配（本工程最老的坑）→ 抖 / 堵转 →
//   保护动作把电机 coast 掉 → "不动且软"。
//
// 真实伺服主站一般 1~2ms 一帧，总线速率 500kbps 完全撑得住；
// 撑不住的是我们的调试打印。所以打印必须限速，而不是让主站减速。
static uint32_t s_rx_print_ms  = 0;   // "收到 0x2xx" 那行
static uint32_t s_tgt_print_ms[2] = { 0, 0 };  // "节点X 目标 ..." 那行
static uint32_t s_rej_print_ms[2] = { 0, 0 };  // "收到目标但不接受" 那行

static bool co_print_ok(uint32_t& last, uint32_t gap_ms) {
  const uint32_t now = millis();
  if ((uint32_t)(now - last) < gap_ms) return false;
  last = now;
  return true;
}

// 默认 TPDO 周期 20ms：够看清，又不至于把 1kHz 的控制环拖慢。
// 真实伺服一般配 1~2ms，靠 SYNC 帧同步；本版用内部定时器代替 SYNC（见 co_pdo_tick）。
#define CO_TPDO_PERIOD_MS 20

static CoNode g_node[2] = {
  { CO_NODE_M0, CO_ST_PREOP, 1000, 0, CIA_SWITCH_DIS, 0, 0, CO_TPDO_PERIOD_MS, 0,
    0, 0, false, 0, 0, 0, 0 },
  { CO_NODE_M1, CO_ST_PREOP, 1000, 0, CIA_SWITCH_DIS, 0, 0, CO_TPDO_PERIOD_MS, 0,
    0, 0, false, 0, 0, 0, 0 },
};

// ------------------------------------------------------------
// 帧追踪：把每帧的原始字节打出来。
//   ★ 这是理解 CANopen 最快的方式 —— 你能直接看到
//     "谁在哪个 ID 上、发了哪 8 个字节、对方回了什么"。
//   只在自检/手动发帧时打开；正常开车时关着（开着会拖慢串口，也没必要）。
// ------------------------------------------------------------
static bool co_trace = false;

// 对象编号 → 人话。SDO 帧的第 2、3 字节就是这个编号（小端）。
// 这些编号不是我们编的，是 CiA 301 / CiA 402 标准规定的，查表即可，不需要背。
static const char* co_obj_name(uint16_t idx) {
  switch (idx) {
    case 0x1000: return "设备类型";
    case 0x1001: return "错误寄存器";
    case 0x1005: return "SYNC 的帧 ID(本机只收不发)";
    case 0x1006: return "SYNC 周期(微秒，0=关)";
    case 0x1016: return "心跳监控(盯着谁、多久算掉线)";
    case 0x1017: return "心跳周期(ms)";
    case 0x1018: return "设备名称(厂商/型号/版本)";
    case 0x1010: return "保存参数(写 save 才认)";
    case 0x1011: return "恢复出厂(写 load 才认)";
    case 0x603F: return "错误码(上一次故障是什么)";
    case 0x6040: return "控制字(主站写它来开关电机)";
    case 0x6041: return "状态字(从站报自己在第几档)";
    case 0x6064: return "实际位置(0.01度)";
    case 0x606C: return "实际速度";
    case 0x607A: return "目标位置(0.01度)";
    case 0x1400: return "RPDO1 通信参数(COB-ID/传输类型)";
    case 0x1600: return "RPDO1 映射表(这帧装了什么)";
    case 0x1800: return "TPDO1 通信参数(COB-ID/传输类型)";
    case 0x1A00: return "TPDO1 映射表(这帧装了什么)";
    default:     return "未实现的对象";
  }
}

// 错误码 → 人话。CiA 301 的错误码用高位分大类：
//   0x2xxx 电流  0x3xxx 电压  0x4xxx 温度  0x5xxx 硬件  0x6xxx 软件  0x8xxx 监控
// ★ 注意：0x603F 和 EMCY 报的是同一个码 —— 一个是"你去查"，一个是"我主动喊"。
static const char* co_err_name(uint16_t c) {
  switch (c) {
    case 0x0000: return "无故障";
    case 0x2310: return "过流";
    case 0x3210: return "母线过压";
    case 0x3220: return "母线欠压";
    case 0x4210: return "过温";
    case 0x8110: return "位置超差";
    default:     return "厂商自定义";
  }
}

// 状态码 → CiA 301 标准名称（与任何 CANopen 主站软件显示的一致）
static const char* co_state_name(uint8_t st) {
  switch (st) {
    case 0x00: return "Boot-up";
    case 0x04: return "Stopped";
    case 0x05: return "Operational";
    case 0x7F: return "Pre-operational";
    default:   return "Unknown";
  }
}

static const char* co_nmt_name(uint8_t c) {
  switch (c) {
    case 0x01: return "Start Remote Node";
    case 0x02: return "Stop Remote Node";
    case 0x80: return "Enter Pre-operational";
    case 0x81: return "Reset Node";
    case 0x82: return "Reset Communication";
    default:   return "Unknown";
  }
}

// ------------------------------------------------------------
// 帧日志：**原始字节为主，解码为辅**
//   第一行 = 抓包格式（跟 CAN 分析仪 / candump 一致，可直接贴给别人）
//   第二行 = 解码，缩进成小字；发 'ol' 可关掉，只留纯帧
// ------------------------------------------------------------
static bool co_annot = true;          // 是否打印解码行

static void co_hex(const uint8_t* d, uint8_t len) {
  for (uint8_t i = 0; i < len; i++) Serial.printf(" %02X", d[i]);
}

static void co_log_frame(const char* dir, uint32_t id, const uint8_t* d, uint8_t len) {
  if (!co_trace || len == 0) return;

  // ---- 第一行：原始帧 ----
  Serial.printf("[CO] %s 0x%03lX [%u]", dir, (unsigned long)id, (unsigned)len);
  co_hex(d, len);
  Serial.println();
  if (!co_annot) return;

  // ---- 第二行：解码 ----
  // NMT：0x000
  if (id == 0x000u) {
    Serial.printf("     └ NMT %s，目标节点 %u\n",
                  co_nmt_name(d[0]), (unsigned)(len > 1 ? d[1] : 0));
    return;
  }

  // EMCY 紧急报文：0x080 + 节点号（第 8 步）。
  //   ★ 和心跳最容易混：心跳是"我还活着"（定期发、1 字节）；
  //     EMCY 是"我出事了"（事件触发、只发一次、8 字节带错误码）。
  //   byte0-1 错误码（小端）  byte2 错误寄存器  byte3-7 厂商自定义
  if (id >= 0x081u && id <= 0x0FFu) {
    const uint16_t code = (uint16_t)(d[0] | ((len > 1 ? d[1] : 0) << 8));
    Serial.printf("     └ EMCY 紧急报文 ← 节点%lu  错误码 0x%04X（%s）  错误寄存器 0x%02X\n",
                  (unsigned long)(id - 0x080u), code, co_err_name(code),
                  (unsigned)(len > 2 ? d[2] : 0));
    return;
  }

  // 心跳：0x700 + 节点号
  if (id >= 0x701u && id <= 0x77Fu) {
    Serial.printf("     └ 心跳 节点%lu  state=0x%02X %s\n",
                  (unsigned long)(id - 0x700u), d[0], co_state_name(d[0]));
    return;
  }

  // PDO：过程数据。和 SDO 最大的区别是"没有人问"——
  //   TPDO（0x180+n）= 从站自己往外喊；RPDO（0x200+n）= 主站往里塞。
  //   帧里没有命令字、没有对象编号，只有"货"，因为装什么早已由映射表定死。
  if (id >= 0x180u && id <= 0x1FFu && len >= 6) {
    const unsigned node = (unsigned)(id - 0x180u);
    const uint16_t sw  = (uint16_t)(d[0] | (d[1] << 8));
    const int32_t  pos = (int32_t)((uint32_t)d[2] | ((uint32_t)d[3] << 8) |
                                   ((uint32_t)d[4] << 16) | ((uint32_t)d[5] << 24));
    Serial.printf("     └ TPDO1 ← 节点%u  状态字=0x%04X %s  实际位置=%.2f°\n",
                  node, sw, cia_state_from_sw(sw), (double)pos / 100.0);
    return;
  }
  if (id >= 0x200u && id <= 0x27Fu && len >= 6) {
    const unsigned node = (unsigned)(id - 0x200u);
    const uint16_t cw  = (uint16_t)(d[0] | (d[1] << 8));
    const int32_t  pos = (int32_t)((uint32_t)d[2] | ((uint32_t)d[3] << 8) |
                                   ((uint32_t)d[4] << 16) | ((uint32_t)d[5] << 24));
    Serial.printf("     └ RPDO1 → 节点%u  控制字=0x%04X  目标位置=%.2f°%s\n",
                  node, cw, (double)pos / 100.0,
                  g_sync_us ? "   （已缓存，等 SYNC）" : "");
    return;
  }

  // SYNC：0x080，全网广播的"节拍"。它自己不带数据，作用只有一个——
  //   让所有从站在同一刻把缓存的目标一起生效，消掉"两条帧一前一后"的先后差。
  if (id == 0x080u) {
    Serial.printf("     └ SYNC 第 %u 拍：缓存的目标此刻一起生效\n", d[0]);
    return;
  }

  // SDO 请求（0x600+n）/ 应答（0x580+n）
  if (id >= 0x580u && id <= 0x6FFu && len >= 4) {
    const uint8_t  cs  = d[0];
    const uint16_t idx = (uint16_t)(d[1] | (d[2] << 8));
    const uint8_t  sub = d[3];
    const uint32_t v   = (uint32_t)d[4] | ((uint32_t)d[5] << 8) |
                         ((uint32_t)d[6] << 16) | ((uint32_t)d[7] << 24);
    const uint32_t node = (id < 0x600u) ? (id - 0x580u) : (id - 0x600u);

    if (cs == 0x40) {
      Serial.printf("     └ SDO 读请求 → 节点%lu", (unsigned long)node);
    } else if (cs == 0x2F || cs == 0x2B || cs == 0x27 || cs == 0x23) {
      Serial.printf("     └ SDO 写请求 → 节点%lu", (unsigned long)node);
    } else if (cs == 0x4F || cs == 0x4B || cs == 0x43) {
      Serial.printf("     └ SDO 读应答 ← 节点%lu", (unsigned long)node);
    } else if (cs == 0x60) {
      Serial.printf("     └ SDO 写应答 ← 节点%lu", (unsigned long)node);
    } else if (cs == 0x80) {
      Serial.printf("     └ SDO 中止   ← 节点%lu", (unsigned long)node);
    } else {
      Serial.printf("     └ SDO 节点%lu", (unsigned long)node);
    }

    // 对象编号 + 子索引 + 名称
    Serial.printf("  0x%04X", idx);
    if (sub) Serial.printf(":%02X", sub);
    Serial.printf(" %s", co_obj_name(idx));

    // 值：先给十进制，再给十六进制，再给物理单位
    if (cs == 0x2F || cs == 0x2B || cs == 0x27 || cs == 0x23) {
      Serial.printf(" = %ld (0x%04lX)", (long)(int32_t)v, (unsigned long)v);
      if (idx == 0x1017) Serial.printf(" = %lu ms", (unsigned long)v);
    } else if (cs == 0x4F || cs == 0x4B || cs == 0x43) {
      Serial.printf(" = %ld (0x%04lX)", (long)(int32_t)v, (unsigned long)v);
      if (idx == 0x6041) {
        Serial.printf(" = %s", cia_state_from_sw((uint16_t)v));
      } else if (idx == 0x6064 || idx == 0x607A) {
        Serial.printf(" = %.2f°", (double)((int32_t)v) / 100.0);
      } else if (idx == 0x1017) {
        Serial.printf(" = %lu ms", (unsigned long)v);
      }
    } else if (cs == 0x80) {
      Serial.printf("  abort 0x%08lX", (unsigned long)v);
    } else if (cs != 0x40 && cs != 0x60) {
      Serial.printf("  命令字 0x%02X", cs);
    }
    Serial.println();
    return;
  }
}

// ------------------------------------------------------------
// CiA 402：状态字怎么算、控制字怎么推状态
// ------------------------------------------------------------
// 状态字 0x6041 只用低 7 位表达"我在第几档"，主站就是靠它判断能不能发运动指令：
//   bit0 Ready to switch on   bit1 Switched on   bit2 Operation enabled
//   bit3 Fault                bit4 Voltage enabled
//   bit6 Switch on disabled   bit9 Remote（"我听总线的"）
// ------------------------------------------------------------
// 第 8 步：EMCY 紧急报文
//   帧 ID = 0x080 + 节点号，8 字节。驱动器出事时"主动喊一嗓子"。
//   ★ 别和心跳混：心跳是"我还活着"（定期、1 字节）；EMCY 是"我出事了"
//     （事件触发、只发一次、8 字节带错误码）。主站不在场就错过了，
//     所以真机还配 0x1003 错误历史，事后能翻。
// ------------------------------------------------------------
static void emcy_send(int ni, uint16_t code) {
  // byte2 是"错误寄存器"，按大类置位：bit0 通用 / bit1 电流 / bit2 电压 / bit3 温度
  uint8_t reg = 0x00;
  if (code) {
    reg = 0x01;
    if (code == 0x2310)                        reg |= 0x02;
    else if (code >= 0x3200 && code < 0x3300)  reg |= 0x04;
    else if (code >= 0x4200 && code < 0x4300)  reg |= 0x08;
  }
  uint8_t d[8] = { (uint8_t)(code & 0xFF), (uint8_t)(code >> 8), reg, 0, 0, 0, 0, 0 };
  const uint32_t id = 0x080u + g_node[ni].node;
  if (co_trace) {
    co_log_frame("TX", id, d, 8);
  } else {
    Serial.printf("[CO] TX 0x%03lX [8] %02X %02X %02X 00 00 00 00 00  ← EMCY 错误码 0x%04X\n",
                  (unsigned long)id, d[0], d[1], d[2], code);
  }
  foc_can_send(id, d, 8);
}

static uint16_t cia_status_word(int ni) {
  uint16_t sw = 0x0210;                     // bit9 远控 + bit4 母线有电（简化：恒 1）
  switch (g_node[ni].cia) {
    case CIA_SWITCH_DIS:  sw |= 0x0040; break;   // → 0x0250
    case CIA_READY:       sw |= 0x0021; break;   // → 0x0231
    case CIA_SWITCHED_ON: sw |= 0x0023; break;   // → 0x0233
    case CIA_OP_ENABLED:  sw |= 0x0027; break;   // → 0x0237
    case CIA_FAULT:       sw |= 0x0008; break;   // → 0x0218
    default: break;                              // → 0x0210
  }
  // bit10 到位标志：只在"已使能"档判断，其余档不谈运动
  if (g_node[ni].cia == CIA_OP_ENABLED) {
    // ★ 必须 ×57.2958 转成度，再 ×100 变成 0.01° 单位，才能和 target 相比。
    //   foc_control_get_full_near() 返回的是**弧度**；漏掉这一步，
    //   65.91° 会算成 1.15，减去 target（6589）永远是 -6500 量级 → bit10 一辈子不亮。
    //   之所以以前偶尔见到 0x0637，是因为目标恰好是 0、电机又停在原点附近，凑巧成立。
    const float err = foc_control_get_full_near(ni) * 57.2957795f * 100.0f - (float)g_node[ni].target;
    if (err < 30.0f && err > -30.0f) sw |= 0x0400;      // bit10 Target reached（0.3° 内）
  }

  return sw;
}

// 控制字 0x6040：只认 CiA 402 最核心的 5 条命令，够把状态机推起来。
//   0x0006 Shutdown          0x0007 Switch on
//   0x000F Enable operation  0x0000 Disable voltage
//   0x0080 Fault reset
// ★ 非法跳转一律"原地不动"：这就是为什么主站必须按顺序发，不能一步到位。
// ★ 第 8 步改了两处：
//   ① Fault reset 按 **bit7 上升沿** 判，不再按电平。按电平的后果是：
//      主站若把 0x0080 一直挂着，故障一出现就被清掉，等于故障根本锁不住。
//   ② Fault 是 **锁存** 态：进去以后除了 bit7 上升沿，别的控制字全无效。
//      这是 CiA 402 最硬的一条规矩 —— 故障没排除，主站不能靠"断一下电
//      再上电"糊弄回去（以前的代码发 ow0 就能溜出 Fault，是个漏洞）。
static void cia_apply_cw(int ni, uint16_t cw) {
  CoNode& nd = g_node[ni];
  const uint8_t old     = nd.cia;
  const bool    rise7   = (cw & 0x0080) != 0 && (nd.control & 0x0080) == 0;
  const bool    changed = (cw != nd.control);
  nd.control = cw;                          // 先存，下一拍才判得出上升沿

  if (nd.cia == CIA_FAULT) {
    if (rise7) {
      nd.cia      = CIA_SWITCH_DIS;         // 复位后只能回"未使能"，不能直接回使能
      nd.err_code = 0;
      // ★ 只改 CANopen 的状态字不够：底层保护模块还停在 coast（PWM 关着），
      //   不一起清掉，主站这边看着"故障没了"，电机却永远没力 —— 就是这次踩的坑。
      foc_clear_fault(ni);
      Serial.printf("[CO] CiA402 节点%u: Fault -> Switch on disabled  (0x6040 bit7 上升沿复位)\n",
                    nd.node);
      emcy_send(ni, 0x0000);                // 错误码 0 = 故障已清除，也报一帧
    } else if (changed) {
      Serial.printf("[CO] 节点%u 在 Fault 里，控制字 0x%04X 被忽略（只有 bit7 上升沿能出来）\n",
                    nd.node, cw);
    }
    return;
  }

  if (cw & 0x0080) {                        // 没故障时收到 Fault reset：只表示"允许复位"
    ;                                       // 不改状态，这是标准行为
  } else if ((cw & 0x0002) == 0) {          // bit1 = 0 → 断电压，回起点
    nd.cia = CIA_SWITCH_DIS;
  } else {
    switch (cw & 0x000F) {
      case 0x06:                            // Shutdown
        nd.cia = CIA_READY;
        break;
      case 0x07:                            // Switch on（也用于 Operation enabled 降档）
        if (nd.cia == CIA_READY || nd.cia == CIA_OP_ENABLED) nd.cia = CIA_SWITCHED_ON;
        break;
      case 0x0F:                            // Enable operation
        if (nd.cia == CIA_SWITCHED_ON)     nd.cia = CIA_OP_ENABLED;
        else if (nd.cia != CIA_OP_ENABLED)
          Serial.printf("[CO] CiA402 节点%u: 现在是 %s，不能直接 Enable（先发 ow7）\n",
                        nd.node, cia_state_name(nd.cia));
        break;
      default: break;
    }
  }

  if (nd.cia != old) {
    // 离开"已使能"这一档 → 立刻就地停住。
    // 本版用"锁住当前角度"代替真正的断 PWM：两块电机的 EN 是同一根线
    //（硬件只有一个 ENABLE），单独断一个做不到。所以语义是"不再走、原地保持"，
    // 不是 coast。真要 coast 得改硬件（每轴一根 EN）。
    if (old == CIA_OP_ENABLED) {
      const float deg = foc_control_get_full_near(ni) * 57.2957795f;
      foc_set_mode(ni, MODE_POSITION);
      foc_set_target_pos(ni, deg * 0.01745329252f);
      nd.target = (int32_t)(deg * 100.0f);
      Serial.printf("[CO] 节点%u 退出 Operation enabled，就地保持 %.2f°\n", nd.node, deg);
    }
    // ★ 第 8 步补：进"已使能"时把模式设回位置环。
    //   故障时我们把模式切成了电流环 0A，不复位就发位置会"发不动 0.5A 全变热"。
    //   ★ 再补一条：底层故障标志也要一起清。底层停在 coast 时，
    //     上层把状态字写成 0x0637 也只是"嘴上说使能"，电机照样是软的。
    if (nd.cia == CIA_OP_ENABLED) {
      foc_clear_fault(ni);
      const float deg = foc_control_get_full_near(ni) * 57.2957795f;
      foc_set_mode(ni, MODE_POSITION);
      foc_set_target_pos(ni, deg * 0.01745329252f);
      nd.target = (int32_t)(deg * 100.0f);
    }
    Serial.printf("[CO] CiA402 节点%u: %s -> %s  (0x6040=0x%04X, 0x6041=0x%04X)\n",
                  nd.node, cia_state_name(old), cia_state_name(nd.cia),
                  cw, cia_status_word(ni));
  }
}

// 把某轴推进 Fault。
//   真机这时候会断继电器、封 PWM 输出。本版两块电机共用一根 ENABLE，
//   单独断一个做不到，所以退一步：切到电流环、给 0 A ——
//   电机失去保持力矩（手能推动），但不会像"就地保持"那样一直顶着电流发热。
//   ★ 这也是个物理事实：故障后电机"变软"，不是坏了，是它真的没劲了。
static void co_set_fault(int ni, uint16_t code) {
  CoNode& nd = g_node[ni];
  if (nd.cia == CIA_FAULT) return;

  nd.cia      = CIA_FAULT;
  nd.err_code = code;
  foc_set_mode(ni, MODE_CURRENT);
  foc_set_target_iq(ni, 0.0f);

  Serial.printf("[CO] ★ 节点%u 报故障 0x%04X（%s）\n", nd.node, code, co_err_name(code));
  Serial.printf("[CO]   状态字 0x%04X，bit3=1；电机已失去保持力矩（手能推动）\n",
                cia_status_word(ni));
  emcy_send(ni, code);
}

// ---------- 对象字典里的常量 ----------
#define CO_DEVICE_TYPE   0x00020192UL   // 0x0002=伺服驱动器，0x0192=CiA 402 协议
#define CO_VENDOR_ID     0x00004859UL   // 'H''Y'
#define CO_PRODUCT_CODE  0x0000005AU   // 双轴云台
#define CO_REVISION      0x00010000UL   // v1.0
#define CO_SERIAL_NO     0x0000A001UL

// SDO abort code
#define CO_ABORT_NO_OBJ      0x06020000UL   // 对象不存在
#define CO_ABORT_READ_ONLY   0x06010002UL   // 只写对象，不能读
#define CO_ABORT_WRITE_ONLY  0x06010001UL   // 只读对象，不能写
#define CO_ABORT_SUB_UNKNOWN 0x06090011UL
#define CO_ABORT_CS          0x05040001UL   // 命令字不认识
#define CO_ABORT_STATE       0x08000022UL   // 设备当前状态不允许这个操作（CiA 402 常用）
#define CO_ABORT_SIGNATURE   0x06040043UL   // 签名不对（0x1010/0x1011 专用）

// ============================================================
// 第 10 步：参数落盘（NVS）
//
//   到第 9 步为止，改过的参数全在 RAM 里：拔掉 USB 再插回来，心跳又变 1000ms，
//   监控又关了。真驱动器不会这样 —— 主站配一次，往后每次上电都该记住。
//
//   CiA 301 给了两个专门干这件事的对象：
//     0x1010 Store parameters     写 0x65766173（"save" 的 ASCII）→ 存
//     0x1011 Restore default      写 0x64616F6C（"load" 的 ASCII）→ 恢复出厂
//   ★ 为什么是"写一个魔数"而不是"写 1"：防止手滑。主站误发一个 0x0001
//     不该把整机参数重置掉，必须明明白白敲出那 4 个字母才算数。
//
//   存哪：ESP32 的 NVS 分区（和 foc_config 存零位是同一个机制，独立命名空间）。
//   注意寿命：NVS 有磨损均衡，但仍按"万次"计，所以只能由主站显式触发，
//   绝对不能放进 loop 里"每次变化就存"。
// ============================================================
#define CO_NVS_VER   1          // 参数结构改了就把这个 +1，老数据自动作废
#define CO_SIG_SAVE  0x65766173UL   // "save"
#define CO_SIG_LOAD  0x64616F6CUL   // "load"

static Preferences co_prefs;
static bool        co_nvs_ready = false;

static void co_nvs_open() {
  if (co_nvs_ready) return;
  co_nvs_ready = co_prefs.begin("canopen", false);
  if (!co_nvs_ready) Serial.println("[CO] NVS 打开失败，参数将无法保存");
}

// 把两个节点的可配参数写成一行，保存和读回都用它，免得两处文案不一致
static void co_param_line(int i, const char* tag) {
  const CoNode& nd = g_node[i];
  Serial.printf("[CO]   %s 节点%u：心跳 %lu ms / TPDO %lu ms / ",
                tag, nd.node, (unsigned long)nd.hb_period, (unsigned long)nd.pdo_period);
  if (nd.mon_ms) Serial.printf("盯 0x%03X 超时 %lu ms\n",
                               0x700u + nd.mon_node, (unsigned long)nd.mon_ms);
  else           Serial.println("不监控主站");
}

static void co_nvs_save(const char* why) {
  co_nvs_open();
  if (!co_nvs_ready) return;

  co_prefs.putUChar("ver", CO_NVS_VER);
  for (int i = 0; i < 2; i++) {
    char k[8];
    snprintf(k, sizeof(k), "hb%d",  i); co_prefs.putULong(k, g_node[i].hb_period);
    snprintf(k, sizeof(k), "pdo%d", i); co_prefs.putULong(k, g_node[i].pdo_period);
    snprintf(k, sizeof(k), "mn%d",  i); co_prefs.putUChar(k, g_node[i].mon_node);
    snprintf(k, sizeof(k), "mt%d",  i); co_prefs.putULong(k, g_node[i].mon_ms);
  }

  Serial.printf("[CO] ===== 参数已写入 NVS（%s）=====\n", why);
  co_param_line(0, "存");
  co_param_line(1, "存");
  Serial.println("[CO] 判断：拔电再上电，这些值会自己回来——那才算真的记住了");
}

static void co_nvs_load() {
  co_nvs_open();
  const uint8_t ver = co_nvs_ready ? co_prefs.getUChar("ver", 0) : 0;
  if (ver != CO_NVS_VER) {
    Serial.println("[CO] NVS 里没有存过参数（或版本不符），本次用默认值");
    Serial.println("[CO] 判断：默认 = 心跳 1000ms / TPDO 20ms / 不监控；改完想留住就发 ost");
    return;
  }

  for (int i = 0; i < 2; i++) {
    char k[8];
    snprintf(k, sizeof(k), "hb%d",  i); g_node[i].hb_period  = co_prefs.getULong(k, g_node[i].hb_period);
    snprintf(k, sizeof(k), "pdo%d", i); g_node[i].pdo_period = co_prefs.getULong(k, g_node[i].pdo_period);
    snprintf(k, sizeof(k), "mn%d",  i); g_node[i].mon_node   = co_prefs.getUChar(k, g_node[i].mon_node);
    snprintf(k, sizeof(k), "mt%d",  i); g_node[i].mon_ms     = co_prefs.getULong(k, g_node[i].mon_ms);
    // ★ mon_last 也要从"现在"开始数。它是 static 变量，上电初值是 0，
    //   而 millis() 已经跑到几万 → now - 0 直接大于超时值，
    //   于是上电第一拍就判"主站掉线"，白刷一行 ⚠ 警告（2026-11-23 补）。
    g_node[i].mon_last   = millis();
  }
  Serial.println("[CO] ===== 从 NVS 读回上次存的参数 =====");
  co_param_line(0, "读");
  co_param_line(1, "读");
  Serial.println("[CO] 判断：这是上次 ost 那一下存进去的，不是编译进去的默认值");
}

// 恢复出厂：内存回默认 + 擦掉 NVS。
//   规范里 restore 只管 RAM，要落盘还得再 store 一次；本版做成一步，
//   否则你会遇到"恢复了、重启又回来"的怪事。
static void co_nvs_restore(const char* why) {
  for (int i = 0; i < 2; i++) {
    g_node[i].hb_period  = 1000;
    g_node[i].pdo_period = CO_TPDO_PERIOD_MS;
    g_node[i].mon_node   = 0;
    g_node[i].mon_ms     = 0;
    g_node[i].hb_last    = millis();
  }
  co_nvs_open();
  if (co_nvs_ready) co_prefs.clear();

  Serial.printf("[CO] ===== 已恢复出厂默认（%s）=====\n", why);
  co_param_line(0, "默认");
  co_param_line(1, "默认");
  Serial.println("[CO] NVS 已擦除：现在拔电重启也是这套默认值");
}

// ------------------------------------------------------------
// 对象字典：查一张"小表"就够了。返回 true = 存在
//   *writable 告诉调用者这个对象能不能写（写只读对象要回 abort）
// ------------------------------------------------------------
static bool od_lookup(int ni, uint16_t idx, uint8_t sub,
                      uint32_t* val, uint8_t* size, bool* writable) {
  *writable = false;

  if (idx == 0x1000 && sub == 0) { *val = CO_DEVICE_TYPE;   *size = 4; return true; }
  if (idx == 0x1001 && sub == 0) { *val = g_node[ni].err_code ? 1u : 0u; *size = 1; return true; }  // 错误寄存器

  // 心跳周期：可写。改它就等于改"每隔多久报一次活着"
  if (idx == 0x1017 && sub == 0) {
    *val = g_node[ni].hb_period; *size = 2; *writable = true; return true;
  }

  // ---------- 第 6 步：SYNC 与心跳监控 ----------
  // 0x1005 SYNC 的 COB-ID。bit30 = 1 表示"我这个设备只收 SYNC、不产生 SYNC"，
  //   所以标准值写成 0x40000080（0x80 是 ID，bit30 是那个"我不发"标记）。
  if (idx == 0x1005 && sub == 0) { *val = 0x40000080UL; *size = 4; return true; }

  // 0x1006 通信周期（µs）。写 0 = 关掉 SYNC（默认）；写非 0 = 进入 SYNC 模式。
  if (idx == 0x1006 && sub == 0) { *val = g_sync_us; *size = 4; *writable = true; return true; }

  // 0x1016 心跳消费：让本节点盯着"某个节点"的心跳，超时就当主站掉线。
  //   值 = (节点号 << 16) | 超时毫秒，0 = 不监控。
  if (idx == 0x1016 && sub == 0) { *val = 2; *size = 1; return true; }
  if (idx == 0x1016 && (sub == 1 || sub == 2)) {
    *val = ((uint32_t)g_node[ni].mon_node << 16) | (g_node[ni].mon_ms & 0xFFFFu);
    *size = 4; *writable = true; return true;
  }

  // ---------- 第 10 步：参数存储 ----------
  //   0x1010:01 读回来永远是 1 = "我支持保存全部参数"（这是规范定义的"能力位"，
  //   不是你上次存的什么值）。真设备里 0x1010:01 还可能多几位，表示分几组存。
  if (idx == 0x1010 && sub == 0) { *val = 1; *size = 1; return true; }
  if (idx == 0x1010 && sub == 1) { *val = 1; *size = 4; *writable = true; return true; }
  if (idx == 0x1011 && sub == 0) { *val = 1; *size = 1; return true; }
  if (idx == 0x1011 && sub == 1) { *val = 1; *size = 4; *writable = true; return true; }

  // 0x603F 错误码（第 8 步）：CiA 402 规定"上一次故障是什么"，2 字节、只读。
  //   和 EMCY 帧里的错误码是同一个数 —— 一个是你去查（拉），一个是它主动喊（推）。
  if (idx == 0x603F && sub == 0) { *val = g_node[ni].err_code; *size = 2; return true; }

  // 0x6040 控制字：可读可写（读回来就是主站上次写的那串，便于核对）
  if (idx == 0x6040 && sub == 0) {
    *val = g_node[ni].control; *size = 2; *writable = true; return true;
  }

  // 0x6041 状态字：只读，由状态机实时算出来
  if (idx == 0x6041 && sub == 0) {
    *val = cia_status_word(ni); *size = 2; return true;
  }

  // 0x607A 目标位置：可读可写（0.01 度）。写它 = 让电机走到这个角度。
  if (idx == 0x607A && sub == 0) {
    *val = (uint32_t)g_node[ni].target; *size = 4; *writable = true; return true;
  }

  // ★ 这是"新增一个 CANopen 对象"的完整示范：只改这一处。
  //   0x6064 = 实际位置（CiA 402 标准定义），单位取 0.01°（SDO 只能传整数，
  //   所以要放大 100 倍再取整）。ni 是节点下标：0=节点1=M0，1=节点2=M1。
  if (idx == 0x6064 && sub == 0) {
    // ★ foc_control_get_full_near() 内部已经乘过 user_sign 了（返回的就是用户坐标），
    //   这里绝对不能再乘一次：(-1)*(-1)=+1，M1 的符号会被乘回去，
    //   表现成"发了 1p30，or2 却读回 -29.97"。踩过。
    *val  = (uint32_t)(int32_t)(foc_control_get_full_near(ni) * 57.2958f * 100.0f);
    *size = 4;
    return true;               // 不设 writable → 只读
  }

  // 设备名（identity）：sub0 = 后面有几个子项
  if (idx == 0x1018 && sub == 0) { *val = 4;               *size = 1; return true; }
  if (idx == 0x1018 && sub == 1) { *val = CO_VENDOR_ID;    *size = 4; return true; }
  if (idx == 0x1018 && sub == 2) { *val = CO_PRODUCT_CODE; *size = 4; return true; }
  if (idx == 0x1018 && sub == 3) { *val = CO_REVISION;     *size = 4; return true; }
  if (idx == 0x1018 && sub == 4) { *val = CO_SERIAL_NO;    *size = 4; return true; }

  // ---------- PDO 配置（第 4 步新增）：只读 ----------
  //   PDO 帧里没有对象编号，主站怎么知道 8 个字节分别是什么？
  //   靠这两张"映射表"：0x1600 说 RPDO1 装了啥，0x1A00 说 TPDO1 装了啥。
  //   值 = (索引 << 16) | (子索引 << 8) | 位长
  //     0x60410010 → 0x6041 子0，16 位
  //     0x607A0020 → 0x607A 子0，32 位
  //   真实驱动器这两组可写（主站能重排映射），本版写死，只报不改。
  if (idx == 0x1400 && sub == 0) { *val = 2;    *size = 1; return true; }   // RPDO1 通信参数
  if (idx == 0x1400 && sub == 1) { *val = 0x200u + g_node[ni].node; *size = 4; return true; }  // COB-ID
  if (idx == 0x1400 && sub == 2) { *val = 0xFF; *size = 1; return true; }  // 传输类型
  if (idx == 0x1600 && sub == 0) { *val = 2;    *size = 1; return true; }   // RPDO1 映射条目数
  if (idx == 0x1600 && sub == 1) { *val = 0x60400010UL; *size = 4; return true; }  // 控制字
  if (idx == 0x1600 && sub == 2) { *val = 0x607A0020UL; *size = 4; return true; }  // 目标位置

  if (idx == 0x1800 && sub == 0) { *val = 3;    *size = 1; return true; }   // TPDO1 通信参数
  if (idx == 0x1800 && sub == 1) { *val = 0x180u + g_node[ni].node; *size = 4; return true; }  // COB-ID
  if (idx == 0x1800 && sub == 2) { *val = 0xFF; *size = 1; return true; }  // 传输类型
  // 0x1800:05 Event timer（ms）：标准里"事件定时器"，含义就是本机每隔多久自动发一次
  //   TPDO，和我们内部的 pdo_period 是同一件事。给它一个标准入口，主站改它
  //   跟改我们内部变量等价 —— 这也是第 10 步要存盘的第三个参数。
  if (idx == 0x1800 && sub == 5) { *val = g_node[ni].pdo_period; *size = 2; *writable = true; return true; }
  if (idx == 0x1A00 && sub == 0) { *val = 2;    *size = 1; return true; }   // TPDO1 映射条目数
  if (idx == 0x1A00 && sub == 1) { *val = 0x60410010UL; *size = 4; return true; }  // 状态字
  if (idx == 0x1A00 && sub == 2) { *val = 0x60640020UL; *size = 4; return true; }  // 实际位置

  return false;
}

static bool od_store(int ni, uint16_t idx, uint8_t sub, uint32_t val) {
  if (idx == 0x1017 && sub == 0) {
    g_node[ni].hb_period = val & 0xFFFFu;
    g_node[ni].hb_last   = millis();
    return true;
  }
  // 0x1006：0 = 关 SYNC（RPDO 收到就生效）；非 0 = 开 SYNC（先缓存，等 0x080）
  if (idx == 0x1006 && sub == 0) {
    g_sync_us = val;
    Serial.printf("[CO] 0x1006 = %lu µs → %s\n", (unsigned long)val,
                  val ? "SYNC 模式：RPDO 先缓存，收到 0x080 才生效"
                      : "无 SYNC：RPDO 收到就立刻生效（第 4 步的老行为）");
    return true;
  }
  // 0x1016：设置"盯谁的心跳"。(节点号 << 16) | 超时毫秒
  if (idx == 0x1016 && (sub == 1 || sub == 2)) {
    g_node[ni].mon_node = (uint8_t)(val >> 16);
    g_node[ni].mon_ms   = val & 0xFFFFu;
    g_node[ni].mon_last = millis();          // 从现在开始数
    Serial.printf("[CO] 0x1016:%u → 节点%u 盯 0x%03X 的心跳，%lu ms 不来就算掉线\n",
                  (unsigned)sub, g_node[ni].node,
                  0x700u + g_node[ni].mon_node, (unsigned long)g_node[ni].mon_ms);
    return true;
  }
  // 0x6040 控制字：不是简单存起来，而是"写进去就推动状态机"
  if (idx == 0x6040 && sub == 0) {
    cia_apply_cw(ni, (uint16_t)val);
    return true;
  }

  // ---------- 第 10 步：0x1010 存 / 0x1011 恢复出厂 ----------
  //   签名校验在 sdo_server 里做（那里已经解析出 val，错了好回 abort），
  //   能走到这里的都是签名正确的请求。
  if (idx == 0x1010 && sub == 1) {
    co_nvs_save("主站写了 0x1010 = save");
    return true;
  }
  if (idx == 0x1011 && sub == 1) {
    co_nvs_restore("主站写了 0x1011 = load");
    return true;
  }

  // 0x1800:05 事件定时器 = TPDO 自动上报周期
  if (idx == 0x1800 && sub == 5) {
    g_node[ni].pdo_period = val & 0xFFFFu;
    g_node[ni].pdo_last   = millis();
    Serial.printf("[CO] 节点%u TPDO 周期 = %lu ms（0 = 不发，等主站来问）\n",
                  g_node[ni].node, (unsigned long)g_node[ni].pdo_period);
    Serial.println("[CO] 提醒：这只是在 RAM 里改了，想断电不丢要再写一次 0x1010 = save");
    return true;
  }
  // 0x607A 目标位置：写进去不是存着，而是立刻下发到三环（等价于串口 p 命令）
  if (idx == 0x607A && sub == 0) {
    CoNode& nd = g_node[ni];
    foc_notify_cmd(ni);                               // 同上：SDO 写目标也算"主站还在"
    nd.target = (int32_t)val;
    const float deg = (float)nd.target / 100.0f;      // 0.01 度 → 度

    // ---- 自动判定 CSP（2026-11-24，画圆时两轴"跟不上、变形"的根因）----
    // 主站每 20ms 喂一个新目标，说明它自己在做插值（CiA 402 的 CSP 模式）。
    // 我们这层"位置软启动"会把每个目标再拉成 0.36 秒的 S 曲线，
    // 结果就是两条滤波器串联：轨迹每 20ms 被重启一次、永远走不到终点，
    // 稳态落后几十度 → 圆变成一小段弧，看起来像"不动"。
    // 所以：两帧间隔 < 200ms 视为流式 → 关掉平滑（给多少跟多少）；
    //       静默超过 200ms 后下一条命令自动恢复平滑（+/-/a/g 的手感不变）。
    static uint32_t s_tgt_ms[2] = { 0, 0 };
    static bool     s_csp[2]    = { false, false };
    const uint32_t now_ms = millis();
    const bool streaming  = ((uint32_t)(now_ms - s_tgt_ms[ni]) < 200u);
    s_tgt_ms[ni] = now_ms;
    if (streaming != s_csp[ni]) {
      s_csp[ni] = streaming;
      foc_set_smooth(ni, !streaming);
    }

    foc_set_mode(ni, MODE_POSITION);
    foc_set_target_pos(ni, deg * 0.01745329252f);     // 入参是 rad，内部自己乘 user_sign
    if (!g_tgt_quiet && co_print_ok(s_tgt_print_ms[ni], 200)) {
      Serial.printf("[CO] 节点%u 目标 %.2f°，当前 %.2f°\n",
                    nd.node, deg, foc_control_get_full_near(ni) * 57.2957795f);
    }
    return true;
  }
  return false;
}

// ------------------------------------------------------------
// 发一帧 SDO 应答
// ------------------------------------------------------------
static void sdo_reply(int ni, uint8_t cs, uint16_t idx, uint8_t sub,
                      uint32_t val, uint8_t size) {
  (void)size;   // 帧长固定 8，保留参数只为可读性
  uint8_t d[8] = { cs,
                   (uint8_t)(idx & 0xFF), (uint8_t)(idx >> 8), sub,
                   (uint8_t)(val & 0xFF), (uint8_t)((val >> 8) & 0xFF),
                   (uint8_t)((val >> 16) & 0xFF), (uint8_t)((val >> 24) & 0xFF) };
  // expedited 帧统一发满 8 字节（未用的字节补 0），主站解析时不用处理短帧
  const bool ok = foc_can_send(0x580u + g_node[ni].node, d, 8);
  // ★ 回帧也打一行：主站说"没收到回复"时，唯一能分清是
  //   "我没回" 还是 "我回了但发不出去（mailbox 满 / 总线无 ACK）" 的办法。
  if (!co_trace) {
    Serial.printf("[CO] => 回复 0x%03lX [8]", (unsigned long)(0x580u + g_node[ni].node));
    co_hex(d, 8);
    Serial.printf("  %s\n", ok ? "OK" : "★发送失败(mailbox满或总线无ACK)");
  }
}

static void sdo_abort(int ni, uint16_t idx, uint8_t sub, uint32_t code) {
  uint8_t d[8] = { 0x80,
                   (uint8_t)(idx & 0xFF), (uint8_t)(idx >> 8), sub,
                   (uint8_t)(code & 0xFF), (uint8_t)((code >> 8) & 0xFF),
                   (uint8_t)((code >> 16) & 0xFF), (uint8_t)((code >> 24) & 0xFF) };
  foc_can_send(0x580u + g_node[ni].node, d, 8);
}

// ------------------------------------------------------------
// SDO 服务器：一次只处理一个对象（没有分段传输，8 字节够用）
//   请求 cs：0x40=读   0x2F/0x2B/0x27/0x23=写(1/2/3/4 字节)
//   应答 cs：0x4F/0x4B/0x43=读回(1/2/4 字节)   0x60=写成功
// ------------------------------------------------------------
static void sdo_server(int ni, const uint8_t* d, uint8_t len) {
  if (len < 8) { sdo_abort(ni, 0, 0, CO_ABORT_CS); return; }

  // ★ 收到 SDO 也等于"主站还活着"。见 rpdo_apply 里同一句的解释。
  if (g_node[ni].mon_ms) g_node[ni].mon_last = millis();

  const uint8_t  cs  = d[0];
  const uint16_t idx = (uint16_t)(d[1] | (d[2] << 8));
  const uint8_t  sub = d[3];

  uint32_t val  = 0;
  uint8_t  size = 0;
  bool     wr   = false;
  const bool exists = od_lookup(ni, idx, sub, &val, &size, &wr);

  // ---- 读 ----
  if (cs == 0x40) {
    if (!exists)               { sdo_abort(ni, idx, sub, CO_ABORT_NO_OBJ);      return; }
    if (size == 0)             { sdo_abort(ni, idx, sub, CO_ABORT_WRITE_ONLY);  return; }
    const uint8_t rcs = (size == 4) ? 0x43 : (size == 2 ? 0x4B : 0x4F);
    sdo_reply(ni, rcs, idx, sub, val, size);
    return;
  }

  // ---- 写 ----
  if (cs == 0x2F || cs == 0x2B || cs == 0x27 || cs == 0x23) {
    const uint8_t wsize = (cs == 0x2F) ? 1 : (cs == 0x2B ? 2 : (cs == 0x27 ? 3 : 4));
    val = 0;
    for (uint8_t i = 0; i < wsize; i++) val |= ((uint32_t)d[4 + i]) << (8 * i);

    if (!exists)  { sdo_abort(ni, idx, sub, CO_ABORT_NO_OBJ);     return; }
    if (!wr)      { sdo_abort(ni, idx, sub, CO_ABORT_READ_ONLY);  return; }
    // CiA 402 的核心约束：没到"已使能"这一档，一律不接受运动指令。
    // 这不是我们加的门槛，是标准规定的主站必须遵守的顺序。
    if (idx == 0x607A && sub == 0 && g_node[ni].cia != CIA_OP_ENABLED) {
      Serial.printf("[CO] CiA402 节点%d: 现在是 %s，不接受 0x607A（先 owf 到 Operation enabled）\n",
                    g_node[ni].node, cia_state_name(g_node[ni].cia));
      sdo_abort(ni, idx, sub, CO_ABORT_STATE);
      return;
    }
    // 第 10 步：0x1010/0x1011 必须是"签名"才算数（防手滑）。
    //   写错不报 0x06090011（子索引不存在），而是 0x06040043（数据本身不对），
    //   因为对象是在的、只是你给的值不是那 4 个字母。
    if (idx == 0x1010 && sub == 1 && val != CO_SIG_SAVE) {
      Serial.printf("[CO] 拒绝保存：签名 0x%08lX ≠ 0x65766173（\"save\"）\n", (unsigned long)val);
      sdo_abort(ni, idx, sub, CO_ABORT_SIGNATURE);
      return;
    }
    if (idx == 0x1011 && sub == 1 && val != CO_SIG_LOAD) {
      Serial.printf("[CO] 拒绝恢复：签名 0x%08lX ≠ 0x64616F6C（\"load\"）\n", (unsigned long)val);
      sdo_abort(ni, idx, sub, CO_ABORT_SIGNATURE);
      return;
    }
    if (!od_store(ni, idx, sub, val)) { sdo_abort(ni, idx, sub, CO_ABORT_SUB_UNKNOWN); return; }
    sdo_reply(ni, 0x60, idx, sub, 0, 0);
    return;
  }

  sdo_abort(ni, idx, sub, CO_ABORT_CS);
}

// ------------------------------------------------------------
// PDO：过程数据（第 4 步）
//
//   一句话区别：SDO 是"一问一答"（2 帧），PDO 是"单向广播"（1 帧）。
//
//   SDO 帧要带命令字 + 对象编号 + 子索引（8 字节里有 4 个是"地址"）；
//   PDO 帧里这些全都没有，8 个字节纯是"货" —— 因为"哪个字节是什么"
//   早被映射表 0x1600 / 0x1A00 定死了，双方照表读。
//
//   代价：不灵活，换内容要改映射。
//   收益：快。1ms 一帧，伺服的位置环就是靠它跑的；SDO 做不到。
// ------------------------------------------------------------

// 发一帧 TPDO1 → 0x180+节点，6 字节 = 状态字(2) + 实际位置(4)
static void pdo_send_tpdo(int ni) {
  const uint16_t sw  = cia_status_word(ni);
  const int32_t  pos = (int32_t)(foc_control_get_full_near(ni) * 57.2958f * 100.0f);
  uint8_t d[6] = { (uint8_t)(sw & 0xFF), (uint8_t)(sw >> 8),
                   (uint8_t)(pos & 0xFF),        (uint8_t)((pos >> 8) & 0xFF),
                   (uint8_t)((pos >> 16) & 0xFF),(uint8_t)((pos >> 24) & 0xFF) };
  co_log_frame("TX", 0x180u + g_node[ni].node, d, 6);
  foc_can_send(0x180u + g_node[ni].node, d, 6);
}

// ============================================================
// 双轴时序统计（2026-11-24）
//
// 要回答的问题：主站每 20ms 发两帧（0x201 给 yaw、0x202 给 pitch），
//   这两帧在总线上永远有先后。**差多少？换算成角度是多少？**
//   只有量出来，才能回答"双轴同步做到什么程度"——
//   否则只能说"我是按顺序发的"，等于没答。
//
// 怎么量：收到一帧 RPDO 就记一次 micros()，等另一轴那帧到了做差。
//   50ms 窗口：超过就当作断档（比如只发了一轴），重新起算，
//   避免把"上一轮的 0x202"和"这一轮的 0x201"配成一对，得出假数据。
//
// 为什么"到达间隔"≈"生效间隔"：不开 SYNC 时 pdo_handle_rpdo 收完立刻
//   调 rpdo_apply → 写 0x607A → 位置环换目标，中间没有缓冲。
//   开了 SYNC 之后这个差会变成 0（两轴在同一帧 SYNC 里连续生效），
//   可以用 oskew 前后对比，作为"SYNC 到底解决了什么"的实测证据。
// ============================================================
static int8_t   s_sk_first_ni = -1;     // 本轮第一帧来自哪个节点（-1 = 没在等）
static uint32_t s_sk_first_us = 0;
static uint32_t s_sk_n   = 0;           // 配成对的轮数
static uint32_t s_sk_sum = 0;
static uint16_t s_sk_min = 0xFFFFu;
static uint16_t s_sk_max = 0;

static void co_skew_note(int ni) {
  const uint32_t now = micros();
  // 第一帧 / 同一轴又来一帧 / 隔太久 → 重新起算
  if (s_sk_first_ni < 0 || ni == s_sk_first_ni ||
      (uint32_t)(now - s_sk_first_us) > 50000u) {
    s_sk_first_ni = (int8_t)ni;
    s_sk_first_us = now;
    return;
  }
  const uint32_t d = (uint32_t)(now - s_sk_first_us);
  s_sk_first_ni = -1;
  if (d < s_sk_min) s_sk_min = (uint16_t)d;
  if (d > s_sk_max) s_sk_max = (uint16_t)d;
  s_sk_sum += d;
  s_sk_n++;
}

static void co_skew_report(void) {
  Serial.println("[CO] ===== 双轴 RPDO 到达间隔 =====");
  if (s_sk_n == 0) {
    Serial.println("[CO] 还没样本。主站要同时在发 0x201 和 0x202（b / a / c 都算）");
  } else {
    const uint32_t avg = s_sk_sum / s_sk_n;
    Serial.printf("[CO] 样本 %lu 轮 | 最小 %u us | 平均 %lu us | 最大 %u us\n",
                  (unsigned long)s_sk_n, (unsigned)s_sk_min,
                  (unsigned long)avg, (unsigned)s_sk_max);
    Serial.printf("[CO] 按 40 deg/s 满速换算：平均 %.3f deg | 最大 %.3f deg\n",
                  (double)avg * 40.0 / 1e6, (double)s_sk_max * 40.0 / 1e6);
    Serial.printf("[CO] 判据：<0.09 deg（12bit 编码器底噪）→ 同步够用，不必上 SYNC\n");
  }
  // 打完清零，下次统计的是"我关心之后"这一段
  s_sk_n = 0; s_sk_sum = 0; s_sk_min = 0xFFFFu; s_sk_max = 0; s_sk_first_ni = -1;
  Serial.println("[CO] （已清零，从现在重新累计）");
}

// 真正把一帧 RPDO 的内容作用到电机上（控制字 → 状态机，目标 → 三环）。
//   单独抽出来，是因为第 6 步的 SYNC 模式要"先缓存、后应用"：
//   缓存时只存字节，等 0x080 一到，两轴调同一个函数、在同一毫秒生效。
static void rpdo_apply(int ni, uint16_t cw, int32_t pos) {
  // ★ 收到 RPDO = 主站还在，喂一次"指令看门狗"。
  //   必须放在下面的"目标没变就 return"之前：CSP 模式下主站每 20ms 发的
  //   目标经常一模一样，但"发了"这件事本身就是心跳，不能因为值相同就当作没收到。
  foc_notify_cmd(ni);
  // ★ 收到 RPDO 也当"主站活着"，必须和 foc_notify_cmd 放一起、放在下面
  //   "目标没变就 return" 之前。
  //   踩过的坑（2026-11-23）：0x1016 配了心跳监控之后，只用 PDO 发指令的主站
  //   （比如我们的 STM32F103）永远不产生 0x70X 心跳，于是监控模块认定"主站掉线"
  //   → 主动降档到 Switch on disabled 并就地锁轴。现象是：使能成功（0x0637）、
  //   电机变硬有劲，但之后每一帧 0x000F 都被拒、一动不动。
  //   SDO 那条路径同理，在 sdo_server 开头也刷一次。
  if (g_node[ni].mon_ms) g_node[ni].mon_last = millis();
  cia_apply_cw(ni, cw);                          // 控制字：推状态机
  if (g_node[ni].cia != CIA_OP_ENABLED) {        // 没使能 → 目标位置不接受
    // 只在新目标时才提示，否则主站每 20ms 一帧会把串口刷爆。
    // 但"没使能 + 主站一直在发新目标"是很常见的组合（比如掉进 Fault 期间），
    // 这时每帧的目标都不一样，"新目标"判断根本挡不住 → 再加一道 500ms 限速。
    if (pos != g_node[ni].target && co_print_ok(s_rej_print_ms[ni], 500)) {
      Serial.printf("[CO] 节点%u 收到目标 %.2f°，但现在是 %s，不接受（先重新使能）\n",
                    g_node[ni].node, (double)((float)pos / 100.0f),
                    cia_state_name(g_node[ni].cia));
    }
    return;
  }
  // 目标没变就不重复下发：真主站每 1ms 送一帧一模一样的，重复下发会把串口刷爆，
  // 也会让轨迹规划器每毫秒重启一次（电机反而抖）。
  if (pos == g_node[ni].target) return;
  od_store(ni, 0x607A, 0, (uint32_t)pos);
}

// 收一帧 RPDO1 ← 0x200+节点，6 字节 = 控制字(2) + 目标位置(4)
//   ★ 这里不查对象字典：PDO 本来就是"直接写对象"，映射表已经说过字节是谁了。
//     但 CiA 402 的门禁（没到 Operation enabled 不许动）必须照旧生效，
//     否则 PDO 会变成一条绕过状态机的后门。
static void pdo_handle_rpdo(int ni, const uint8_t* d, uint8_t len) {
  if (len < 6) return;
  co_skew_note(ni);      // ★ 先记到达时刻：这是双轴时序的唯一原始数据
  const uint16_t cw  = (uint16_t)(d[0] | (d[1] << 8));
  const int32_t  pos = (int32_t)((uint32_t)d[2] | ((uint32_t)d[3] << 8) |
                                 ((uint32_t)d[4] << 16) | ((uint32_t)d[5] << 24));

  // SYNC 模式：先压进"待生效"槽位，等 0x080 那一下再一起动。
  if (g_sync_us) {
    g_node[ni].pend_cw  = cw;
    g_node[ni].pend_pos = pos;
    g_node[ni].has_pend = true;
    return;
  }
  rpdo_apply(ni, cw, pos);
}

// 收到 SYNC（0x080）：把所有节点缓存的目标一起生效。
//   这就是 SYNC 的全部价值 —— 两条 RPDO 在总线上总有先后（差几百微秒），
//   但它们"执行"的时刻由同一帧 SYNC 决定，于是两轴真正同时出发。
static void sync_apply() {
  for (int i = 0; i < 2; i++) {
    if (!g_node[i].has_pend) continue;
    g_node[i].has_pend = false;
    rpdo_apply(i, g_node[i].pend_cw, g_node[i].pend_pos);
  }

  // TPDO 也改成"按 SYNC 发"，才对得上 0x1800:02 报的 0xFF（由 SYNC 触发）。
  // 不开 SYNC 时仍由内部定时器发（co_pdo_tick）。
  if (g_sync_us) {
    for (int i = 0; i < 2; i++) {
      if (g_node[i].state == CO_ST_OPERATIONAL && g_node[i].pdo_period) pdo_send_tpdo(i);
    }
  }
}

// 假主站发一帧 SYNC。回环模式下这帧会自己收回来触发一次 sync_apply，
// 这里再主动调一次，就不依赖回环是否及时（重复调用无害：has_pend 已清零）。
static void sync_send() {
  uint8_t d[1] = { ++g_sync_cnt };
  co_log_frame("TX", 0x080u, d, 1);
  foc_can_send(0x080u, d, 1);
  sync_apply();
}

// 「心跳消费」：盯着某个节点的心跳，超时就当主站掉线。
//   真实伺服必须有这一条：主站电缆断了，电机不能抱着最后一个目标继续顶电流。
//   本版超时动作 = 走 0x0000（Disable voltage）那条路 → 降档 + 就地锁轴。
static void co_consumer_tick() {
  const uint32_t now = millis();
  for (int i = 0; i < 2; i++) {
    CoNode& nd = g_node[i];
    // mon_node == 0 表示"没指定盯谁"，这种情况下 mon_last 永远不会被刷新，
    //   一旦 mon_ms 非 0 就是"必然超时"，纯属配置错误 —— 直接不监控。
    if (nd.mon_ms == 0 || nd.mon_node == 0) continue;
    if ((uint32_t)(now - nd.mon_last) < nd.mon_ms) continue;

    const uint32_t to = nd.mon_ms;
    nd.mon_ms = 0;          // 只报一次，否则每毫秒刷一行
    Serial.printf("[CO] ⚠ 节点%u 心跳超时：%lu ms 没收到 0x%03X → 判定主站掉线，"
                  "降档到 Switch on disabled 并就地锁轴\n",
                  nd.node, (unsigned long)to, 0x700u + nd.mon_node);
    cia_apply_cw(i, 0x0000);
  }
}

// TPDO 定时上报。CiA 301 规定：只有 Operational 状态才允许发 PDO，
// Pre-operational 只能走 SDO —— 所以主站上电必须先发一帧 NMT Start（0x000）。
static void co_pdo_tick() {
  const uint32_t now = millis();
  for (int i = 0; i < 2; i++) {
    if (g_node[i].state != CO_ST_OPERATIONAL) continue;
    // SYNC 模式下 TPDO 由 0x080 触发（见 sync_apply），这里不再按定时器发，
    // 否则同一份数据会被发两遍。
    if (g_sync_us) continue;
    if (g_node[i].pdo_period == 0) continue;
    if ((uint32_t)(now - g_node[i].pdo_last) < g_node[i].pdo_period) continue;
    g_node[i].pdo_last = now;
    pdo_send_tpdo(i);
  }
}

// ------------------------------------------------------------
// NMT：0x000，data[0]=命令 data[1]=目标节点(0=全部)
//   1=Start  2=Stop  0x80=进预操作  0x81=复位节点  0x82=复位通信
// ------------------------------------------------------------
static void nmt_handler(const uint8_t* d, uint8_t len) {
  if (len < 2) return;
  const uint8_t cmd = d[0];
  const uint8_t tgt = d[1];

  for (int i = 0; i < 2; i++) {
    if (tgt != 0 && tgt != g_node[i].node) continue;

    switch (cmd) {
      case 0x01: g_node[i].state = CO_ST_OPERATIONAL; break;
      case 0x02: g_node[i].state = CO_ST_STOPPED;     break;
      case 0x80: g_node[i].state = CO_ST_PREOP;       break;
      case 0x81:   // Reset node：真正做需要重启芯片，这版只回到 Boot-up → 预操作
      case 0x82:   // Reset communication
        g_node[i].state     = CO_ST_PREOP;
        g_node[i].hb_last   = millis();
        g_node[i].cia       = CIA_SWITCH_DIS;   // 复位通信 = 驱动器必须回未使能
        g_node[i].control   = 0;
        g_node[i].pdo_last  = millis();
        // 复位通信 = 通信参数回到"上次保存的值"（不是出厂值）。
        //   这正是第 10 步存盘的意义：没存过的改动会被这次复位冲掉。
        //   （本版两节点共用一套存储，复位任一节点等于两个都重读，这点不如真设备。）
        if (i == 0) co_nvs_load();
        g_node[i].mon_last  = millis();        // 从现在起重新数
        g_node[i].has_pend  = false;
        break;
      default: continue;
    }
    Serial.printf("[CO] NMT -> 节点 %d  state = 0x%02X %s\n",
                  g_node[i].node, g_node[i].state, co_state_name(g_node[i].state));
  }
}

// ------------------------------------------------------------
// 收帧分发
// ------------------------------------------------------------
bool co_on_frame(uint32_t can_id, const uint8_t* data, uint8_t len) {
  // ★ 总线诊断：主循环里真的收到外部帧就打一行。
  //   为什么需要它：接真主站联调时，"对面没反应"有两种完全不同的原因——
  //   ① 我这侧根本没收到（线/波特率/模式）；② 收到了但我没回。
  //   不打印这一行，两种情况在串口上长得一模一样，只能瞎猜。
  //   自检期间（co_trace=true）不打印，否则和 co_xfer 的 TX/RX 重复。
  //   心跳 0x701~0x77F 不打（每秒一条，纯刷屏）。
  //   限速 100ms：主站按 20ms 送 PDO 时，这条会变成 50Hz × 两轴，
  //   单这一行就 4600 B/s，必须掐。单条命令调试时隔很久才来一帧，不受影响。
  if (!co_trace && len > 0 && can_id < 0x700u && co_print_ok(s_rx_print_ms, 100)) {
    Serial.printf("[CO] <= 收到 0x%03lX [%u]", (unsigned long)can_id, (unsigned)len);
    co_hex(data, len);
    Serial.println();
  }

  if (can_id == 0x000u) { nmt_handler(data, len); return true; }

  // SYNC：先处理，它是"让缓存生效"的扳机
  if (can_id == 0x080u) { sync_apply(); return true; }

  // 心跳进来 → 刷新"心跳消费"的计时器。
  //   这一句必须放在"是不是发给我的"判断之前：主站的心跳 ID 是 0x700+主站号，
  //   本来就不是发给节点 1/2 的，但我们要靠它判断主站还活着。
  if (can_id >= 0x701u && can_id <= 0x77Fu) {
    const uint8_t from = (uint8_t)(can_id - 0x700u);
    for (int i = 0; i < 2; i++) {
      if (g_node[i].mon_ms && g_node[i].mon_node == from) g_node[i].mon_last = millis();
    }
    return false;      // 心跳不是给从站的命令，继续往下交给日志
  }

  for (int i = 0; i < 2; i++) {
    if (can_id == 0x600u + g_node[i].node) { sdo_server(i, data, len);        return true; }
    if (can_id == 0x200u + g_node[i].node) { pdo_handle_rpdo(i, data, len);   return true; }
  }

  return false;   // 不是 CANopen 的帧，交回给调用者
}

void co_init() {
  for (int i = 0; i < 2; i++) {
    g_node[i].state     = CO_ST_BOOT;
    g_node[i].hb_period = 1000;
    g_node[i].hb_last   = millis();
    g_node[i].cia       = CIA_SWITCH_DIS;   // CiA 402：上电一律停在"未使能"
    g_node[i].control   = 0;
    g_node[i].pdo_period = CO_TPDO_PERIOD_MS;
    g_node[i].pdo_last   = millis();
  }

  // ★ 第 10 步：先看 NVS 里有没有存过参数，有就覆盖上面这套默认值。
  //   顺序很重要：必须在发 Boot-up 心跳之前读回来，否则第一帧心跳的周期是错的。
  co_nvs_load();

  for (int i = 0; i < 2; i++) {
    uint8_t d[1] = { CO_ST_BOOT };          // Boot-up 报文，告诉主站"我醒了"
    foc_can_send(0x700u + g_node[i].node, d, 1);

    g_node[i].state   = CO_ST_PREOP;        // 上电默认进预操作
    g_node[i].hb_last = millis();
  }
  Serial.println("[CO] CANopen 从站就绪：节点 1 = M0(yaw)，节点 2 = M1(pitch)");
}

// 心跳发送：单独抽成按时间判定的函数。
//   ★ 为什么不能只在 co_update() 里发：自检（co_xfer）是阻塞的，等待期间主循环不跑，
//     心跳就会一帧都发不出去 —— 假主站自然永远等不到 0x701。
//   真实 CANopen 里心跳由"时间"驱动，和主循环忙不忙无关，所以这里用 millis 判定，
//   co_update() 与自检等待循环都调同一个 tick。
static void co_hb_tick() {
  const uint32_t now = millis();
  for (int i = 0; i < 2; i++) {
    if (g_node[i].hb_period == 0) continue;
    if ((uint32_t)(now - g_node[i].hb_last) < g_node[i].hb_period) continue;
    g_node[i].hb_last = now;

    uint8_t d[1] = { g_node[i].state };
    // 方向按「本机视角」标：TX = 从这块板发到总线，RX = 从总线收到。
    // 和 CANoe / PCAN-View 的习惯一致，看日志时不用再想"谁发谁收"。
    co_log_frame("TX", 0x700u + g_node[i].node, d, 1);
    foc_can_send(0x700u + g_node[i].node, d, 1);
  }
}

// ------------------------------------------------------------
// 第 9 步：把固件自己的保护接到 CiA 402 上
//   以前是两层皮：底层 foc_protection 说"我过流了"（打印 [FAULT]），
//   而 CiA 402 状态字还老老实实报 0x0237 —— 主站被蒙在鼓里。
//   真驱动器不可能这样。这里每 loop 对一次表：谁真坏了，状态字立刻变 0x0218，
//   EMCY 立刻发出去，不用等主站来问。
//
//   方向是单向的：底层 → 上层。反方向不做（主站不能靠写控制字把硬件故障"写没"）。
//   清除也只能走一条路：底层先好（0r），主站再按复位（ow80）。
// ------------------------------------------------------------
static uint16_t co_map_fault(int bits) {
  // 固件是位标志（可叠加），CiA 是"一个错误码"，按严重程度取一个报
  if (bits & 0x01) return 0x2310;   // FAULT_OVERCURRENT 过流
  if (bits & 0x02) return 0x7120;   // FAULT_STALL       堵转
  if (bits & 0x04) return 0x8611;   // FAULT_POS_ERROR   跟随误差过大（CiA 402 标准码）
  if (bits & 0x08) return 0x8100;   // FAULT_CMD_TIMEOUT 指令超时
  return 0xFF00;                    // 兜底：厂商自定义
}

static void co_fault_sync() {
  for (int i = 0; i < 2; i++) {
    const int bits = foc_get_fault(i);
    if (bits && g_node[i].cia != CIA_FAULT) co_set_fault(i, co_map_fault(bits));
  }
}

void co_update() {
  // CANopen 是"状态保持"协议：使能之后主站不需要周期性发命令，
  // 它自己保持上一次给的指令。但底层保护里的 CMD_TIMEOUT（100s 没收到指令就报故障）
  // 是按"串口逐条命令"的思路设计的，在这里会把静止不动的从站误判成故障并 coast 掉。
  // 所以使能期间每 100ms 替主站续一次"指令有效"；真正的掉线由心跳消费（0x1016）负责。
  static uint32_t s_keep = 0;
  const uint32_t now = millis();
  if ((uint32_t)(now - s_keep) >= 100u) {
    s_keep = now;
    for (int i = 0; i < 2; i++)
      if (g_node[i].cia == CIA_OP_ENABLED) foc_notify_cmd(i);
  }

  co_fault_sync(); co_hb_tick(); co_pdo_tick(); co_consumer_tick();
}

// ============================================================
// 自检：板内假主站
//   把 MCP2515 切到回环模式（自己发的帧自己收回），用标准帧去问协议栈，
//   也就是"没有 CAN 分析仪也能验证 CANopen 收发通路"。
// ============================================================

// 发一帧请求，等指定 ID 的应答；路上收到的请求帧照样交给协议栈处理
static bool co_xfer(uint32_t req_id, const uint8_t* req, uint8_t len,
                    uint32_t resp_id, uint8_t* resp, uint32_t timeout_ms) {
  co_log_frame("TX", req_id, req, len);
  if (!foc_can_send(req_id, req, len)) return false;

  const uint32_t t0 = millis();
  while ((uint32_t)(millis() - t0) < timeout_ms) {
    uint32_t id = 0;
    uint8_t  d[8];
    uint8_t  dl = 0;
    while (foc_can_recv(&id, d, &dl)) {
      if (id == resp_id) {
        co_log_frame("RX", id, d, dl);
        for (int i = 0; i < 8; i++) resp[i] = (i < dl) ? d[i] : 0;
        return true;
      }
      co_on_frame(id, d, dl);     // 回环里也会收到自己发的请求 → 让协议栈应答
    }
    co_hb_tick();                 // 阻塞期间照常发心跳（否则第 4 步永远等不到 0x701）
    delay(1);
  }
  return false;
}

static uint32_t rd32(const uint8_t* p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

// ============================================================
// PDO 演示（串口 op / op2）
//   为什么需要它：PDO 是"没人提问"的帧，光靠自己发 SDO 看不到它。
//   这里让板子既当从站（发 TPDO、收 RPDO）又当假主站（发 RPDO），
//   全程零条 SDO，走完 使能→定位→到位 的全部流程。
// ============================================================

// 发一帧 RPDO1（6 字节：控制字 2 + 目标位置 4）
static void pdo_send_rpdo(uint8_t node, uint16_t cw, int32_t target) {
  uint8_t d[6] = { (uint8_t)(cw & 0xFF), (uint8_t)(cw >> 8),
                   (uint8_t)((uint32_t)target & 0xFF),
                   (uint8_t)(((uint32_t)target >> 8) & 0xFF),
                   (uint8_t)(((uint32_t)target >> 16) & 0xFF),
                   (uint8_t)(((uint32_t)target >> 24) & 0xFF) };
  co_log_frame("TX", 0x200u + node, d, 6);
  foc_can_send(0x200u + node, d, 6);
}

// 切回环后，接收缓冲里可能还压着正常模式期间的旧帧，先倒掉。
// 不倒的话日志里会冒出"上一条命令的回声"（比如刚发 NMT，下面却先收到一条 0x7F 心跳）。
static void pdo_drain() {
  uint32_t id = 0;
  uint8_t  d[8];
  uint8_t  dl = 0;
  uint16_t guard = 0;
  while (foc_can_recv(&id, d, &dl) && guard++ < 64) {}
}

// 边等边收：把这段时间内总线上出现的帧全交给协议栈处理，顺便打日志。
//   skip_a / skip_b：刚发出去的两条 RPDO，回环会把它们原样收回来，不用重复打印。
static void pdo_pump2(uint32_t ms, uint32_t skip_a, uint32_t skip_b) {
  const uint32_t t0 = millis();
  while ((uint32_t)(millis() - t0) < ms) {
    uint32_t id = 0;
    uint8_t  d[8];
    uint8_t  dl = 0;
    while (foc_can_recv(&id, d, &dl)) {
      // 回环模式会把本机刚发出去的帧原样收回来：
      //   0x180~0x1FF 是我们自己发的 TPDO，0x700~0x77F 是心跳，0x080 是 SYNC，
      //   skip_a/skip_b 是刚发的 RPDO。这些回声不重复打印（TX 那行已经打过了），
      //   但仍然交给协议栈处理。
      const bool echo = (id >= 0x180u && id <= 0x1FFu) ||
                        (id >= 0x700u && id <= 0x77Fu) ||
                        (id == 0x080u) ||
                        (id == skip_a) || (id == skip_b);
      if (!echo) co_log_frame("RX", id, d, dl);
      co_on_frame(id, d, dl);
    }
    // ★ 关键：这一行不加，演示期间电机一动不动。
    //   因为整段演示是阻塞的，主循环里的 foc_control_update() 停摆 ——
    //   位置环不跑、编码器不刷新，TPDO 报的自然永远是同一个角度。
    foc_control_update();
    co_hb_tick();
    co_consumer_tick();   // 阻塞期间也要查心跳超时，否则演示里永远不触发
    co_pdo_tick();      // 阻塞期间照常自动上报（跟心跳同一个道理）
    delay(1);
  }
}

static void pdo_pump(uint32_t ms, uint32_t skip_id) {
  pdo_pump2(ms, skip_id, 0xFFFFFFFFu);
}

static void co_pdo_demo(uint8_t node) {
  const int ni = node - 1;

  // 目标位置取"往中间走 15°"：两轴的机械中心都在 90°，永远朝中间动就不会撞挡块。
  const float   cur    = foc_control_get_full_near(ni) * 57.2957795f;
  const float   tgt    = (cur > 90.0f) ? (cur - 15.0f) : (cur + 15.0f);
  const int32_t tgt100 = (int32_t)(tgt * 100.0f + (tgt >= 0 ? 0.5f : -0.5f));

  Serial.println();
  Serial.printf("[CO] ===== PDO 演示：节点%u（%s）=====\n",
                (unsigned)node, node == 1 ? "M0 yaw" : "M1 pitch");
  Serial.printf("[CO] 当前 %.2f°，目标 %.2f°\n", (double)cur, (double)tgt);
  Serial.println("[CO] 这一次全程零条 SDO。0x201 是 RPDO（主站→从站，装 控制字+目标位置）");
  Serial.println("[CO]                              0x181 是 TPDO（从站→主站，装 状态字+实际位置）");

  if (!foc_can_set_loopback(true)) { Serial.println("[CO] 切回环失败"); return; }
  delay(5);
  pdo_drain();                      // 倒掉正常模式下残留的旧帧

  const uint32_t bak_period = g_node[ni].pdo_period;
  g_node[ni].pdo_period = 150;      // 演示期间放慢：20ms 一条会把屏幕刷掉
  g_node[ni].pdo_last   = millis();
  co_trace = true;

  // 1) NMT start —— 预操作状态下从站不许发 PDO，必须先被"启动"
  {
    uint8_t d[2] = { 0x01, node };
    co_log_frame("TX", 0x000u, d, 2);
    foc_can_send(0x000u, d, 2);
    pdo_pump(60, 0x000u);
  }

  // 2) 三步使能，全部用 RPDO 送控制字（对比：SDO 要 3 次往返，这里 3 帧单向）
  pdo_send_rpdo(node, 0x0006, 0);         pdo_pump(170, 0x200u + node);
  pdo_send_rpdo(node, 0x0007, 0);         pdo_pump(170, 0x200u + node);
  pdo_send_rpdo(node, 0x000F, 0);         pdo_pump(170, 0x200u + node);

  // 3) 同一帧里带上目标位置 → 电机开始走
  //   先把 target 预置成一个不可能的值，逼它一定下发一次
  //   （pdo_handle_rpdo 里"目标没变就跳过"，否则演示到一半目标刚好相同就不动了）
  g_node[ni].target = (int32_t)0x7FFFFFFF;
  Serial.println("[CO] ---- 下面这一帧带目标位置，电机开始动 ----");
  pdo_send_rpdo(node, 0x000F, tgt100);    pdo_pump(180, 0x200u + node);

  // 4) 看实际位置一路追上目标
  Serial.println("[CO] ---- TPDO 自动上报中，看「实际位置」追到目标 ----");
  pdo_pump(900, 0x200u + node);

  co_trace = false;
  foc_can_set_loopback(false);
  // 演示完收回预操作：PDO 只在 Operational 才发，这样总线立刻安静下来。
  // （没有真主站在总线上时，一直发 TPDO 会因收不到 ACK 而反复重发）
  g_node[ni].state = CO_ST_PREOP;
  g_node[ni].pdo_period = bak_period;
  g_node[ni].pdo_last   = millis();

  Serial.printf("[CO] ===== 演示结束：实际 %.2f°，目标 %.2f° =====\n",
                (double)(foc_control_get_full_near(ni) * 57.2957795f), (double)tgt);
}

// ============================================================
// 双轴 PDO 联动（串口 oj / osy）
//
//   前面 `op` 一次只驱动一个节点。云台要的是"两轴同时动、同时到"——
//   主站每一轮同时给 0x201 和 0x202 各发一帧 RPDO，两个从站各自按轨迹走到位。
//
//   这才是 CANopen 用在机构上的样子：不是"问一句动一下"，
//   而是"每一轮把整台机器的目标全推下去"。
//
//   `oj`  = 两条 RPDO 直接生效（帧发出即有先后，差几百微秒）
//   `osy` = 每轮多一发 0x080 SYNC，两条目标在同一拍变生效（工业多轴的标准做法）
// ============================================================
static float co_deg(int ni) { return foc_control_get_full_near(ni) * 57.2957795f; }

// 把角度夹进限位内侧 5°，避免贴边撞挡块
static float co_clamp_deg(float v, float lo, float hi) {
  if (lo > hi) { const float t = lo; lo = hi; hi = t; }   // 防御：万一存反了
  if (v < lo + 5.0f) v = lo + 5.0f;
  if (v > hi - 5.0f) v = hi - 5.0f;
  return v;
}

// 航点振幅：最多用到可用宽度的 1/4，保证 ±amp 都离限位有足够余量。
static float co_stage_amp(float lo, float hi, float want) {
  if (lo > hi) { const float t = lo; lo = hi; hi = t; }
  const float half = (hi - lo) * 0.25f;
  if (want > half) want = half;
  if (want < 3.0f) want = 3.0f;
  return want;
}

static int32_t co_deg100(float deg) {
  return (int32_t)(deg >= 0 ? (deg * 100.0f + 0.5f) : (deg * 100.0f - 0.5f));
}

// 给一个节点推 3 帧使能控制字（0x06 → 0x07 → 0x0F）
//   ★ 帧里带的目标位置必须是"当前位置"，不能写 0。
//     写 0 = 一边使能一边命令它回原点：上一版就是这么干的，
//     日志里 yaw 26.46° 被使能帧拽到 0°，用户看到的是"我还没发航点它就动了"。
//   ★ SYNC 版必须"先收进缓存，再打拍子"：
//     回环模式下 RPDO 是发出去再收回来才进缓存的，SYNC 若比回声先到，
//     缓存里就是空的 → 目标永远不生效（这个顺序踩过一次）。
static void dual_enable(uint8_t node, bool use_sync) {
  const int ni = node - 1;
  const int32_t hold = co_deg100(co_deg(ni));    // 就地保持
  g_node[ni].target = (int32_t)0x7FFFFFFF;       // 逼目标位置一定会下发

  const uint16_t cws[3] = { 0x0006, 0x0007, 0x000F };
  for (int k = 0; k < 3; k++) {
    pdo_send_rpdo(node, cws[k], hold);
    if (use_sync) {
      pdo_pump(8, 0x200u + node);   // 先让这一帧进缓存
      sync_send();                  // 再打拍子，缓存才生效
    }
    pdo_pump(30, 0x200u + node);
  }
}

// 等到两轴都进入 ±0.5° 就算到位；返回耗时（ms），超时返回 -1。
//   不能用 0 表示超时：万一第一轮就已经在目标上，耗时本来就是 0ms。
static int32_t dual_wait_arrive(uint32_t timeout_ms) {
  const uint32_t t0 = millis();
  while ((uint32_t)(millis() - t0) < timeout_ms) {
    pdo_pump(20, 0xFFFFFFFFu);
    bool ok = true;
    for (int i = 0; i < 2; i++) {
      const float err = co_deg(i) - (float)g_node[i].target / 100.0f;
      if (err > 0.5f || err < -0.5f) { ok = false; break; }
    }
    if (ok) return (int32_t)(millis() - t0);
  }
  return -1;
}

static void co_pdo_dual(bool use_sync) {
  const float y0 = co_deg(0), p0 = co_deg(1);
  const float ylo = foc_get_pos_limit_lo(0) * 57.2957795f, yhi = foc_get_pos_limit_hi(0) * 57.2957795f;
  const float plo = foc_get_pos_limit_lo(1) * 57.2957795f, phi = foc_get_pos_limit_hi(1) * 57.2957795f;

  // ★ 航点的基准点：把当前位置"夹进一个能放下 ±amp 的区间"，而不是直接取区间中点。
  //   两个都要避开的坑：
  //   ① 以起点为基准 ±12°：起点在限位外时会被 co_clamp_deg 全钳成 lo+5，
  //      三个航点一模一样，pitch 一整天停在 5.00°（上一版就栽在这）。
  //   ② 直接取中点：从 5° 一步跳到 90° 是 85° 的猛冲，不像"联动"，像"回零"。
  //   正确做法：基准点 = clamp(当前位置, [lo+5+amp, hi-5-amp])，
  //   既保证 ±amp 一定在限位内，又让第一段行程尽量短。
  const float ay = co_stage_amp(ylo, yhi, 12.0f);
  const float ap = co_stage_amp(plo, phi, 12.0f);
  const float yc = constrain(y0, ylo + 5.0f + ay, yhi - 5.0f - ay);
  const float pc = constrain(p0, plo + 5.0f + ap, phi - 5.0f - ap);

  // 4 个航点：到基准 → 对角 ↗ → 对角 ↘ → 回基准
  const float wy[4] = { yc, yc + ay, yc - ay, yc      };
  const float wp[4] = { pc, pc - ap, pc + ap, pc      };

  Serial.println();
  if (use_sync) {
    Serial.println("[CO] ===== 双轴 PDO 联动（SYNC 版）：两条 RPDO 先缓存，0x080 一到同时生效 =====");
  } else {
    Serial.println("[CO] ===== 双轴 PDO 联动：两个节点，两条 RPDO，同步出发 =====");
  }
  Serial.printf("[CO] 起点   yaw %.2f°（限位 %.1f~%.1f）   pitch %.2f°（限位 %.1f~%.1f）\n",
                (double)y0, (double)ylo, (double)yhi, (double)p0, (double)plo, (double)phi);
  // 起点在限位外要明说：固件会把目标钳回边界，使能帧就会把轴拽回去。
  if (y0 < ylo || y0 > yhi) {
    Serial.printf("[CO] 注意 yaw 起点 %.2f° 在限位外，使能帧会先把它拉回 %.1f°\n",
                  (double)y0, (double)co_clamp_deg(y0, ylo, yhi));
  }
  if (p0 < plo || p0 > phi) {
    Serial.printf("[CO] 注意 pitch 起点 %.2f° 在限位外，使能帧会先把它拉回 %.1f°\n",
                  (double)p0, (double)co_clamp_deg(p0, plo, phi));
  }
  Serial.printf("[CO] 航点舞台：yaw %.1f°±%.1f°   pitch %.1f°±%.1f°（都在限位内）\n",
                (double)yc, (double)ay, (double)pc, (double)ap);
  if (!foc_can_set_loopback(true)) { Serial.println("[CO] 切回环失败"); return; }
  delay(5);
  pdo_drain();

  const uint32_t bak0 = g_node[0].pdo_period, bak1 = g_node[1].pdo_period;
  g_node[0].pdo_period = 0;
  g_node[1].pdo_period = 0;      // 联动期间不用自动上报，帧太多反而看不清
  co_trace = true;

  // SYNC 版：先把 0x1006 置非 0，RPDO 才会"缓存而不是立刻生效"。
  const uint32_t bak_sync = g_sync_us;
  if (use_sync) {
    g_sync_us = 20000;          // 写这个值只是让 SYNC 模式生效
    Serial.println("[CO] 0x1006 = 20000 us → SYNC 模式：下面每条 RPDO 都会显示「已缓存」");
  }

  // NMT start 两个节点：不发这一帧，Pre-operational 状态下 PDO 一律不收不发
  {
    uint8_t d[2] = { 0x01, 0 };
    co_log_frame("TX", 0x000u, d, 2);
    foc_can_send(0x000u, d, 2);
    pdo_pump(40, 0x000u);
  }
  Serial.println("[CO] ---- 使能阶段：两个节点各 3 帧 RPDO ----");
  dual_enable(1, use_sync);
  dual_enable(2, use_sync);
  pdo_pump(60, 0xFFFFFFFFu);

  Serial.println("[CO] ---- 航点阶段：每轮同时给 0x201 和 0x202 发目标 ----");
  // oj 只看摘要（帧太多）；osy 要把"缓存 → SYNC → 同时生效"的过程看清楚，所以留帧。
  co_trace = use_sync;

  const uint32_t t_all = millis();
  float worst = 0.0f;
  for (int k = 0; k < 4; k++) {
    const int32_t ty = co_deg100(wy[k]);
    const int32_t tp = co_deg100(wp[k]);
    g_node[0].target = (int32_t)0x7FFFFFFF;
    g_node[1].target = (int32_t)0x7FFFFFFF;

    Serial.printf("[CO] 航点%d  目标 yaw %.2f°(%+.1f°)  pitch %.2f°(%+.1f°)\n",
                  k + 1, (double)wy[k], (double)(wy[k] - co_deg(0)),
                  (double)wp[k], (double)(wp[k] - co_deg(1)));
    pdo_send_rpdo(1, 0x000F, ty);
    pdo_send_rpdo(2, 0x000F, tp);
    // ★ SYNC 版多这一帧：两条目标在上面只是"躺进缓存"，这一帧才让它们同时生效。
    //   顺序不能反：先把两帧收进缓存（8ms 足够，回环几乎是即时的），再打拍子。
    if (use_sync) {
      pdo_pump2(8, 0x201u, 0x202u);
      sync_send();
      Serial.println("[CO]   ↑ 这一拍之后两轴才真正开始动（同一个时间点）");
    }

    // 进入 ±0.5° 判定线：这一刻只是"进门"，不是"停稳"
    const int32_t dt = dual_wait_arrive(4000);
    if (dt >= 0) {
      Serial.printf("[CO]   两轴同时进 ±0.5°：耗时 %.2f s（进门误差 yaw %+.2f°  pitch %+.2f°）\n",
                    (double)dt / 1000.0, (double)(co_deg(0) - wy[k]),
                    (double)(co_deg(1) - wp[k]));
    } else {
      Serial.println("[CO]   超时：4s 内两轴没同时进 0.5°");
    }

    // ★ 连续采 5 点，看清"进了 0.5° 之后到底怎么收敛"。
    //   只采一个点分不清三种情况，而三种的修法完全相反：
    //     · 采样太早    → 序列单调趋 0，最后进死区           → 什么都不用改
    //     · 静摩擦卡住  → 序列卡在某个值不动                 → 要加积分/前馈
    //     · 阻尼不足    → 序列穿过 0 再回来，符号翻转        → 要调 Kd 或速度环 P
    float sy[5], sp[5];
    for (int s = 0; s < 5; s++) {
      pdo_pump(200, 0xFFFFFFFFu);
      sy[s] = co_deg(0) - wy[k];
      sp[s] = co_deg(1) - wp[k];
    }

    Serial.print("[CO]   收敛 yaw  ");
    for (int s = 0; s < 5; s++) Serial.printf("%+.2f ", (double)sy[s]);
    Serial.println("  每 0.2s 一点，0 = 正好在目标");
    Serial.print("[CO]   收敛 pitch");
    for (int s = 0; s < 5; s++) Serial.printf(" %+.2f", (double)sp[s]);
    Serial.println();

    // 一眼判断这 5 个数在说什么（省得每次都要我解释）
    float ymin = sy[0], ymax = sy[0], pmin = sp[0], pmax = sp[0];
    for (int s = 1; s < 5; s++) {
      if (sy[s] < ymin) ymin = sy[s];
      if (sy[s] > ymax) ymax = sy[s];
      if (sp[s] < pmin) pmin = sp[s];
      if (sp[s] > pmax) pmax = sp[s];
    }
    const float yspan = ymax - ymin, pspan = pmax - pmin;
    if (yspan < 0.75f && pspan < 0.75f) {
      Serial.printf("[CO]   判断       两轴波动 %.2f° / %.2f°，都在编码器底噪（0.7°）以内 "
                    "→ 电机其实已经停住，指针只是在抖\n", (double)yspan, (double)pspan);
    } else {
      Serial.printf("[CO]   判断       两轴波动 %.2f° / %.2f°，超出底噪 → 还在真动，看是不是一直朝 0 收敛\n",
                    (double)yspan, (double)pspan);
    }

    const float ey = sy[4];
    const float ep = sp[4];
    if (fabsf(ey) > worst) worst = fabsf(ey);
    if (fabsf(ep) > worst) worst = fabsf(ep);
    const bool conv = (fabsf(ey) < 0.2f && fabsf(ep) < 0.2f);
    Serial.printf("[CO]   终点       yaw %.2f°（%+.2f°）   pitch %.2f°（%+.2f°）   %s\n",
                  (double)co_deg(0), (double)ey, (double)co_deg(1), (double)ep,
                  conv ? "已进 0.2° 死区" : "仍在死区外");
  }

  co_trace = false;
  foc_can_set_loopback(false);
  g_node[0].state = g_node[1].state = CO_ST_PREOP;
  g_node[0].pdo_period = bak0;
  g_node[1].pdo_period = bak1;
  g_node[0].pdo_last = g_node[1].pdo_last = millis();
  g_sync_us = bak_sync;                    // 演示完恢复（默认 0 = 无 SYNC）
  g_node[0].has_pend = g_node[1].has_pend = false;

  Serial.printf("[CO] ===== 联动结束：总耗时 %.2f s，终点最大误差 %.2f° =====\n",
                (double)(millis() - t_all) / 1000.0, (double)worst);
}

// ============================================================
// 第 7 步：连续轨迹（CSP + SYNC）
//
//   前面 op / oj / osy 都是"点到点"：发一个目标，等它到了，再发下一个。
//   真实伺服跑连续轨迹（激光笔画线、机械臂插补）不是这么干的 —
//   主站每 1~20ms 就换一个新目标，驱动器不停顿地跟着走。
//   这个工作模式在 CiA 402 里叫 CSP（周期同步位置，模式 8）。
//
//   和前面的三处不同：
//     ① 关掉位置平滑。每 20ms 换目标，若每段都重新规划 S 曲线，
//        速度会被反复清零，实际永远追不上主站。
//     ② 每拍一帧 SYNC，两轴在同一刻取用同一拍的目标。
//     ③ 统计"跟踪误差"（目标是动的，误差是过程量），不是"到位误差"。
// ============================================================
static void co_pdo_trace() {
  const float y0 = co_deg(0), p0 = co_deg(1);
  const float ylo = foc_get_pos_limit_lo(0) * 57.2957795f, yhi = foc_get_pos_limit_hi(0) * 57.2957795f;
  const float plo = foc_get_pos_limit_lo(1) * 57.2957795f, phi = foc_get_pos_limit_hi(1) * 57.2957795f;

  const float ay = co_stage_amp(ylo, yhi, 10.0f);
  const float ap = co_stage_amp(plo, phi, 10.0f);
  const float yc = constrain(y0, ylo + 5.0f + ay, yhi - 5.0f - ay);
  const float pc = constrain(p0, plo + 5.0f + ap, phi - 5.0f - ap);

  const uint32_t PERIOD = 20;          // 一拍 20ms（真主站常用 1~2ms）
  const uint32_t DUR_MS = 10000;       // 跑 10 秒
  const uint32_t N_TICK = DUR_MS / PERIOD;
  const float    T_SEC  = (float)DUR_MS / 1000.0f;

  Serial.println();
  Serial.println("[CO] ===== 连续轨迹（CSP + SYNC）：主站每 20ms 喂一个新目标 =====");
  Serial.printf("[CO] 起点 yaw %.2f°  pitch %.2f°；绕中心 (%.1f, %.1f) 走，摆幅 ±%.1f° / ±%.1f°\n",
                (double)y0, (double)p0, (double)yc, (double)pc, (double)ay, (double)ap);
  Serial.printf("[CO] 8 字：横轴一个来回，纵轴两个来回，共 %lu 拍 = %.0f 秒\n",
                (unsigned long)N_TICK, (double)T_SEC);
  if (y0 < ylo || y0 > yhi) Serial.println("[CO] 注意 yaw 起点在限位外，先会被拉回边界");
  if (p0 < plo || p0 > phi) Serial.println("[CO] 注意 pitch 起点在限位外，先会被拉回边界");

  if (!foc_can_set_loopback(true)) { Serial.println("[CO] 切回环失败"); return; }
  delay(5);
  pdo_drain();

  const uint32_t bak0 = g_node[0].pdo_period, bak1 = g_node[1].pdo_period;
  g_node[0].pdo_period = 0;
  g_node[1].pdo_period = 0;
  const uint32_t bak_sync = g_sync_us;
  g_sync_us   = PERIOD * 1000;     // 进 SYNC 模式：RPDO 先缓存，等 0x080
  co_trace    = false;             // 轨迹期间不刷帧，只留进度和统计
  g_tgt_quiet = true;

  // ★ CSP：关掉位置平滑 —— 这一步和"点到点"最本质的差别
  foc_set_smooth(0, false);
  foc_set_smooth(1, false);
  Serial.println("[CO] 已关闭位置平滑（CSP 模式）：主站给多少就跟多少，不再自己规划 S 曲线");
  Serial.println("[CO] 跑完会自动恢复，串口 p 命令不受影响");

  {
    uint8_t d[2] = { 0x01, 0 };
    foc_can_send(0x000u, d, 2);
    pdo_pump(40, 0x000u);
  }
  dual_enable(1, true);
  dual_enable(2, true);
  pdo_pump(60, 0xFFFFFFFFu);

  Serial.println("[CO] ---- 开始：每 20ms 一帧 0x201 + 0x202 + 0x080 ----");

  float maxy = 0.0f, maxp = 0.0f, sumy = 0.0f, sump = 0.0f;
  uint32_t per_min = 0xFFFFFFFFu, per_max = 0;
  uint32_t last_send = 0;
  const uint32_t t_start = millis();
  uint32_t next = t_start;
  uint32_t next_report = t_start + 2000;
  uint32_t tick = 0;

  while (tick < N_TICK) {
    // 控制环必须在这段阻塞里自己跑：主循环停着，不跑就没人更新三环
    foc_control_update();
    {
      uint32_t id = 0; uint8_t d[8]; uint8_t dl = 0;
      while (foc_can_recv(&id, d, &dl)) co_on_frame(id, d, dl);
    }
    co_hb_tick();
    co_consumer_tick();
    co_pdo_tick();

    const uint32_t now = millis();
    if ((int32_t)(now - next) >= 0) {
      next += PERIOD;
      if (last_send && now > last_send) {
        const uint32_t dt = now - last_send;
        if (dt < per_min) per_min = dt;
        if (dt > per_max) per_max = dt;
      }
      last_send = now;

      const float t  = (float)(now - t_start) / 1000.0f;
      const float wy = yc + ay * sinf(2.0f * PI * t / T_SEC);
      const float wp = pc + ap * sinf(4.0f * PI * t / T_SEC);

      g_node[0].target = (int32_t)0x7FFFFFFF;   // 逼目标一定下发（不然会被"没变就跳过"吃掉）
      g_node[1].target = (int32_t)0x7FFFFFFF;
      pdo_send_rpdo(1, 0x000F, co_deg100(wy));
      pdo_send_rpdo(2, 0x000F, co_deg100(wp));
      pdo_pump(4, 0x201u);        // 让两条回声都进缓存
      sync_send();                // 这一拍，两轴同时取用

      const float ey = co_deg(0) - wy;
      const float ep = co_deg(1) - wp;
      if (fabsf(ey) > maxy) maxy = fabsf(ey);
      if (fabsf(ep) > maxp) maxp = fabsf(ep);
      sumy += fabsf(ey);
      sump += fabsf(ep);
      tick++;

      if ((int32_t)(now - next_report) >= 0) {
        next_report += 2000;
        Serial.printf("[CO]   %4.1fs  目标(%.1f, %.1f)  实际(%.1f, %.1f)  误差(%+.2f, %+.2f)\n",
                      (double)((now - t_start) / 1000.0f),
                      (double)wy, (double)wp, (double)co_deg(0), (double)co_deg(1),
                      (double)ey, (double)ep);
      }
    }
    delay(1);
  }

  g_tgt_quiet = false;
  foc_set_smooth(0, true);
  foc_set_smooth(1, true);
  g_sync_us = bak_sync;
  g_node[0].pdo_period = bak0;
  g_node[1].pdo_period = bak1;
  g_node[0].state = g_node[1].state = CO_ST_PREOP;
  g_node[0].has_pend = g_node[1].has_pend = false;
  foc_can_set_loopback(false);

  const float n = (tick > 0) ? (float)tick : 1.0f;
  Serial.printf("[CO] ===== 结束：%lu 拍，总耗时 %.2f s，实际周期 %.0f~%.0f ms（目标 %lu ms）=====\n",
                (unsigned long)tick, (double)(millis() - t_start) / 1000.0,
                (double)per_min, (double)per_max, (unsigned long)PERIOD);
  Serial.printf("[CO] 跟踪误差  平均 yaw %.2f° / pitch %.2f°    最大 yaw %.2f° / pitch %.2f°\n",
                (double)(sumy / n), (double)(sump / n), (double)maxy, (double)maxp);
  Serial.println("[CO] 注意：目标一直在动，所以这里的误差是「跟不上多少」（滞后），不是「没到位」");
  Serial.println("[CO] 位置平滑已恢复，可以继续用 p 命令");
}

// 串口 of：制造一次"过流"故障，把第 8 步整条路径走一遍。
//   为什么要"制造"：真过流要烧硬件。这里只是把驱动器切进 Fault 档 + 发一帧 EMCY，
//   主站那边看到的东西和真故障一模一样：状态字 bit3=1、0x603F 有码、收到 0x081。
static void co_inject_fault() {
  Serial.println();
  Serial.println("[CO] ===== 故障演示：模拟节点1 过流 =====");
  Serial.printf("[CO] 故障前状态字 0x%04X（%s）\n",
                cia_status_word(0), cia_state_from_sw(cia_status_word(0)));

  if (!foc_can_set_loopback(true)) { Serial.println("[CO] 切回环失败"); return; }
  delay(5);
  pdo_drain();

  co_set_fault(0, 0x2310);          // 节点1（M0）报 EMCY 0x2310 = 过流
  pdo_pump(30, 0x081u);             // 回环把 EMCY 原样收回来，打一条 RX 日志
  foc_can_set_loopback(false);

  Serial.println("[CO] 接下来试这三条：");
  Serial.println("[CO]   os            看状态字（0x0218，bit3=1）");
  Serial.println("[CO]   o40 3F 60 00  读 0x603F 错误码");
  Serial.println("[CO]   ow80          故障复位（bit7 上升沿）→ 回 0x0250");
}

// 串口 ohb：假装主站还在，发一帧 0x77F（节点 127 的心跳）。
//   为什么要这么绕：总线上只有这块板，没有别人能替我们发心跳；
//   所以借回环模式让这帧绕回来被自己收到 —— 收到就算"主站活着"。
//   用途：配好 0x1016 之后反复发 ohb 保持"主站在线"，一停手就会看到降档。
static void co_fake_hb() {
  if (!foc_can_set_loopback(true)) { Serial.println("[CO] 切回环失败"); return; }
  delay(5);
  pdo_drain();

  uint8_t d[1] = { 0x05 };            // 0x05 = Operational，主站通常报这个
  foc_can_send(0x77Fu, d, 1);
  pdo_pump(30, 0x77Fu);               // 收回来 → co_on_frame 会刷新 mon_last
  foc_can_set_loopback(false);

  Serial.println("[CO] 已发出主站心跳 0x77F [05]（模拟主站还活着）");
  for (int i = 0; i < 2; i++) {
    if (g_node[i].mon_ms == 0 && g_node[i].mon_node == 0) {
      Serial.printf("[CO]   节点%u：没配心跳监控（0x1016 还是 0）\n", g_node[i].node);
      continue;
    }
    const uint32_t used = (uint32_t)(millis() - g_node[i].mon_last);
    Serial.printf("[CO]   节点%u：盯着 0x%03X，超时 %lu ms，已过 %lu ms\n",
                  g_node[i].node, 0x700u + g_node[i].mon_node,
                  (unsigned long)g_node[i].mon_ms, (unsigned long)used);
  }
}

int co_self_test() {  Serial.println("[CO] ===== CANopen 自检（板内回环，假主站）=====");

  int pass = 0;
  uint8_t req[8];
  uint8_t rsp[8];

  // ★ 自检会亲手改 0x1017（第 3 项写 200ms）和 TPDO 周期，所以必须先把原值存下来，
  //   结束再原样还回去。以前是硬编码"恢复成 1000 / 20ms"，等于每次自检都把
  //   用户（或主站）配好的参数冲掉 —— 看着像"NVS 没存住"，实际是自检干的。
  const uint32_t bak_hb  = g_node[0].hb_period;
  const uint32_t bak_pdo0 = g_node[0].pdo_period;
  const uint32_t bak_pdo1 = g_node[1].pdo_period;

  // 自检全程只用 SDO，把 TPDO 自动上报临时关掉，日志才看得清。
  // （第 4 步的 NMT start 会让节点进 Operational，不关的话每 20ms 就冒一行 TPDO）
  for (int i = 0; i < 2; i++) g_node[i].pdo_period = 0;

  if (!foc_can_set_loopback(true)) {
    Serial.println("[CO] 切回环模式失败，自检跳过");
    return 0;
  }
  delay(5);
  co_trace = true;          // 从现在起打印每一帧（原始字节 + 解码）

  // 1) 读设备类型 0x1000
  memset(req, 0, 8);
  req[0] = 0x40; req[1] = 0x00; req[2] = 0x10; req[3] = 0x00;
  if (co_xfer(0x601, req, 8, 0x581, rsp, 200) && rsp[0] == 0x43 &&
      rd32(&rsp[4]) == CO_DEVICE_TYPE) {
    Serial.printf("[CO] 1) SDO 读 0x1000 设备类型 ........ PASS  0x%08lX\n", (unsigned long)rd32(&rsp[4]));
    pass++;
  } else {
    Serial.println("[CO] 1) SDO 读 0x1000 设备类型 ........ FAIL");
  }

  // 2) 读厂商号 0x1018:01
  memset(req, 0, 8);
  req[0] = 0x40; req[1] = 0x18; req[2] = 0x10; req[3] = 0x01;
  if (co_xfer(0x601, req, 8, 0x581, rsp, 200) && rsp[0] == 0x43 &&
      rd32(&rsp[4]) == CO_VENDOR_ID) {
    Serial.printf("[CO] 2) SDO 读 0x1018:01 厂商号 ...... PASS  0x%08lX\n", (unsigned long)rd32(&rsp[4]));
    pass++;
  } else {
    Serial.println("[CO] 2) SDO 读 0x1018:01 厂商号 ...... FAIL");
  }

  // 3) 写心跳周期 0x1017 = 200ms，再读回来
  memset(req, 0, 8);
  req[0] = 0x2B; req[1] = 0x17; req[2] = 0x10; req[3] = 0x00;
  req[4] = 200; req[5] = 0;
  bool w_ok = co_xfer(0x601, req, 8, 0x581, rsp, 200) && rsp[0] == 0x60;

  memset(req, 0, 8);
  req[0] = 0x40; req[1] = 0x17; req[2] = 0x10; req[3] = 0x00;
  uint32_t hb_read = 0;
  bool r_ok = co_xfer(0x601, req, 8, 0x581, rsp, 200) && rsp[0] == 0x4B;
  if (r_ok) hb_read = (uint32_t)rsp[4] | ((uint32_t)rsp[5] << 8);

  if (w_ok && r_ok && hb_read == 200) {
    Serial.printf("[CO] 3) SDO 写 0x1017 = 200ms ....... PASS  回读 %lu ms\n", (unsigned long)hb_read);
    pass++;
  } else {
    Serial.printf("[CO] 3) SDO 写 0x1017 = 200ms ....... FAIL (w=%d r=%d v=%lu)\n",
                  (int)w_ok, (int)r_ok, (unsigned long)hb_read);
  }

  // 4) NMT start 节点 1，然后应该收到 state=0x05 的心跳（周期已改 200ms）
  uint8_t nmt[2] = { 0x01, (uint8_t)CO_NODE_M0 };
  if (co_xfer(0x000, nmt, 2, 0x701, rsp, 600) && rsp[0] == CO_ST_OPERATIONAL) {
    Serial.println("[CO] 4) NMT start 节点1 → 心跳 0x05 .. PASS  Operational");
    pass++;
  } else {
    Serial.printf("[CO] 4) NMT start 节点1 → 心跳 0x05 .. FAIL (state=0x%02X)\n", rsp[0]);
  }

  // 5) 读一个不存在的对象，必须回 abort 而不是瞎编数据
  memset(req, 0, 8);
  req[0] = 0x40; req[1] = 0x00; req[2] = 0x20; req[3] = 0x00;
  if (co_xfer(0x601, req, 8, 0x581, rsp, 200) && rsp[0] == 0x80 &&
      rd32(&rsp[4]) == CO_ABORT_NO_OBJ) {
    Serial.printf("[CO] 5) 读不存在的 0x2000 ........... PASS  abort 0x%08lX\n",
                  (unsigned long)rd32(&rsp[4]));
    pass++;
  } else {
    Serial.println("[CO] 5) 读不存在的 0x2000 ........... FAIL");
  }

  // 收尾：把自检期间改过的参数还原（不是"恢复默认"），回到正常模式
  g_node[0].hb_period = bak_hb;
  g_node[0].hb_last   = millis();
  g_node[0].state     = CO_ST_PREOP;          // 自检把节点1 启动了，收回预操作
  g_node[0].pdo_period = bak_pdo0;
  g_node[1].pdo_period = bak_pdo1;
  for (int i = 0; i < 2; i++) g_node[i].pdo_last = millis();
  co_trace = false;
  foc_can_set_loopback(false);

  Serial.printf("[CO] ===== 自检 %d/5 %s =====\n", pass, pass == 5 ? "全部通过" : "有失败项");
  return pass;
}

// ============================================================
// 手动当主站：o<空格分隔的十六进制字节>
//   例：o40 00 10 00   → 向节点1 的 SDO 发"读 0x1000"，打印发出的帧和收到的应答
//   实现上临时切回环（自己发自己收），用完自动切回正常模式。
//   目的：让你在没有 CAN 分析仪的情况下，亲手摸一遍 CANopen 的字节。
// ============================================================
static uint8_t co_hexval(char c) {
  if (c >= '0' && c <= '9') return (uint8_t)(c - '0');
  if (c >= 'a' && c <= 'f') return (uint8_t)(c - 'a' + 10);
  if (c >= 'A' && c <= 'F') return (uint8_t)(c - 'A' + 10);
  return 0xFF;
}

void co_manual_sdo(const char* hex) {
  uint8_t d[8];
  uint8_t n = 0;
  uint8_t node = 1;                 // 默认节点 1 = M0；写 "o2 ..." 就是节点 2 = M1
  const char* p = hex;

  // ★ 短命令：把固定的 4 个字节替你填好，走的还是同一条 SDO 通路。
  //   发送 'or'  = 读 M0 现在在哪    'or2' = 读 M1 现在在哪
  //   'ol' = 切换解码注解（关掉后只剩纯十六进制帧）
  if (strcmp(hex, "l") == 0 || strcmp(hex, "L") == 0) {
    co_annot = !co_annot;
    Serial.printf("[CO] 解码注解已%s（第一行原始帧始终打印）\n",
                  co_annot ? "开启" : "关闭，纯十六进制模式");
    return;
  }
  if (strcmp(hex, "r") == 0 || strcmp(hex, "R") == 0) {
    d[0] = 0x40; d[1] = 0x64; d[2] = 0x60; d[3] = 0x00;
    n = 4; node = 1;
  } else if (strcmp(hex, "p") == 0 || strcmp(hex, "P") == 0) {
    co_pdo_demo(1); return;               // PDO 演示：M0
  } else if (strcmp(hex, "p2") == 0 || strcmp(hex, "P2") == 0) {
    co_pdo_demo(2); return;               // PDO 演示：M1
  } else if (strcmp(hex, "j") == 0 || strcmp(hex, "J") == 0) {
    co_pdo_dual(false); return;           // 双轴 PDO 联动（无 SYNC）
  } else if (strcmp(hex, "sy") == 0 || strcmp(hex, "SY") == 0) {
    co_pdo_dual(true); return;            // 双轴 PDO 联动（带 SYNC 同步）
  } else if (strcmp(hex, "t") == 0 || strcmp(hex, "T") == 0) {
    co_pdo_trace(); return;               // 连续轨迹：每 20ms 喂一个点（CSP）
  } else if (strcmp(hex, "hb") == 0 || strcmp(hex, "HB") == 0) {
    co_fake_hb(); return;                 // 假装主站还在（测心跳超时用）
  } else if (strcmp(hex, "f") == 0 || strcmp(hex, "F") == 0) {
    co_inject_fault(); return;            // 制造一次故障：Fault / EMCY / 0x603F
  } else if (strcmp(hex, "st") == 0 || strcmp(hex, "ST") == 0) {
    co_nvs_save("串口 ost"); return;       // 存参数（等价于写 0x1010 = "save"）
  } else if (strcmp(hex, "std") == 0 || strcmp(hex, "STD") == 0) {
    Serial.println("[CO] ===== 当前生效的参数（内存）=====");
    co_param_line(0, "现在");
    co_param_line(1, "现在");
    return;
  } else if (strcmp(hex, "stc") == 0 || strcmp(hex, "STC") == 0) {
    co_nvs_restore("串口 ostc"); return;   // 恢复出厂 + 擦 NVS
  } else if (strcmp(hex, "skew") == 0 || strcmp(hex, "SKEW") == 0) {
    co_skew_report(); return;              // 双轴 RPDO 到达间隔（同步到底做到什么程度）
  } else if (strcmp(hex, "r2") == 0 || strcmp(hex, "R2") == 0) {
    d[0] = 0x40; d[1] = 0x64; d[2] = 0x60; d[3] = 0x00;
    n = 4; node = 2;
  } else if (strcmp(hex, "s") == 0 || strcmp(hex, "S") == 0) {
    // 读状态字 0x6041：驱动器现在在第几档
    d[0] = 0x40; d[1] = 0x41; d[2] = 0x60; d[3] = 0x00;
    n = 4; node = 1;
  } else if (strcmp(hex, "s2") == 0 || strcmp(hex, "S2") == 0) {
    d[0] = 0x40; d[1] = 0x41; d[2] = 0x60; d[3] = 0x00;
    n = 4; node = 2;
  } else if ((hex[0] == 'm' || hex[0] == 'M') && hex[1]) {
    // 写目标位置 0x607A：om90 = 90.00°，om-30 = -30.00°，om45.5 也认
    const float deg = atof(hex + 1);
    const int32_t v = (int32_t)(deg >= 0 ? (deg * 100.0f + 0.5f) : (deg * 100.0f - 0.5f));
    d[0] = 0x23; d[1] = 0x7A; d[2] = 0x60; d[3] = 0x00;   // 0x23 = 4 字节写
    d[4] = (uint8_t)(v & 0xFF);
    d[5] = (uint8_t)((v >> 8) & 0xFF);
    d[6] = (uint8_t)((v >> 16) & 0xFF);
    d[7] = (uint8_t)((v >> 24) & 0xFF);
    n = 8; node = 1;
  } else if ((hex[0] == 'w' || hex[0] == 'W') && hex[1]) {
    // 写控制字 0x6040：w6=Shutdown  w7=Switch on  wf=Enable  w0=Disable voltage
    uint32_t cw = 0;
    for (const char* q = hex + 1; *q; q++) {
      const uint8_t v = co_hexval(*q);
      if (v == 0xFF) break;
      cw = (cw << 4) | v;
    }
    d[0] = 0x2B; d[1] = 0x40; d[2] = 0x60; d[3] = 0x00;
    d[4] = (uint8_t)(cw & 0xFF);
    d[5] = (uint8_t)((cw >> 8) & 0xFF);
    n = 6; node = 1;
  }

  while (*p == ' ') p++;
  if ((p[0] == '1' || p[0] == '2') && (p[1] == ' ' || p[1] == ',')) {
    node = (uint8_t)(p[0] - '0');
    p += 2;
  }

  // ★ 必须一直解析到字符串结束（以前写成 while (n == 0 && ...)，
  //   只解析第一个字节就退出，"o40 17 10 00" 实际发出去的是 40 00 00 00，
  //   对象号变成 0x0000 → 从站回 abort 0x06020000，看起来像"对象没实现"）。
  while (*p && n < 8) {
    while (*p == ' ' || *p == ',' || *p == '\t') p++;
    if (!*p) break;
    uint8_t hi = co_hexval(p[0]);
    uint8_t lo = (p[1] ? co_hexval(p[1]) : 0xFF);
    if (hi == 0xFF || lo == 0xFF) break;
    d[n++] = (uint8_t)((hi << 4) | lo);
    p += 2;
  }

  if (n == 0) {
    // ★ 这里同时是"命令不认识"的兜底出口。以前只打用法不打回显，
    //   用户打 osts（真命令是 ostd）时看到一大坨用法，不知道自己打错了哪个字。
    Serial.printf("[CO] 不认识命令 \"o%s\"（下面是对照表）\n", hex);
    Serial.println("[CO] 用法:");
    Serial.println("[CO]   位置/状态:");
    Serial.println("[CO]     or / or2      读 M0 / M1 实际位置（0x6064）");
    Serial.println("[CO]     os / os2      读 M0 / M1 状态字（0x6041，CiA 402 第几档）");
    Serial.println("[CO]   控制字（CiA 402，按顺序发）:");
    Serial.println("[CO]     ow6           Shutdown         -> Ready to switch on");
    Serial.println("[CO]     ow7           Switch on        -> Switched on");
    Serial.println("[CO]     owf           Enable operation -> Operation enabled");
    Serial.println("[CO]     ow0           Disable voltage  -> Switch on disabled");
    Serial.println("[CO]   运动（必须先在 Operation enabled）:");
    Serial.println("[CO]     om90          写目标位置 0x607A = 90.00°（om-30 / om45.5 都认）");
    Serial.println("[CO]   PDO 演示（全程不发 SDO，看 0x201 / 0x181 那两条帧）:");
    Serial.println("[CO]     op            M0：NMT启动→三步使能→走到目标，全用 PDO");
    Serial.println("[CO]     op2           M1 同上");
    Serial.println("[CO]     oj            双轴联动：两轴同时出发走四个航点（推荐先跑这个）");
    Serial.println("[CO]     osy           同上，但每轮多一帧 SYNC(0x080)，两轴在同一拍生效");
    Serial.println("[CO]     ot            连续轨迹：每 20ms 喂一个点，8 字走 10 秒（CSP 模式）");
    Serial.println("[CO]   故障与复位（第 8 步）:");
    Serial.println("[CO]     of            制造一次过流故障（状态字 bit3=1 + EMCY 帧 0x081）");
    Serial.println("[CO]     o40 3F 60 00  读 0x603F 错误码（和 EMCY 里那个数一样）");
    Serial.println("[CO]     ow80          故障复位（必须 bit7 上升沿）-> 回到 0x0250");
    Serial.println("[CO]   安全（主站掉线保护）:");
    Serial.println("[CO]     ohb           假装主站还在，发一帧 0x77F 心跳（测超时用）");
    Serial.println("[CO]     配法: o23 16 10 01 C8 00 7F 00  = 节点127 超时 200ms");
    Serial.println("[CO]   ol              切换解码注解（关掉 = 纯十六进制帧）");
    Serial.println("[CO]   参数持久化（第 10 步，三条都必须带 o）:");
    Serial.println("[CO]     ost           把当前参数存进 NVS（= 写 0x1010:\"save\"）");
    Serial.println("[CO]     ostd          看看现在内存里的值（还没存的话断电就丢）");
    Serial.println("[CO]     ostc          恢复出厂默认并擦掉 NVS");
    Serial.println("[CO]   ★ 为什么必须带 o：s / t / d 本身就是独立命令（查档 / 自检 / 诊断），");
    Serial.println("[CO]     少了 o 会被拆成三条分别执行。o 开头才是 CANopen 命令的入口。");
    Serial.println("[CO]   标准写法（主站视角，走 SDO）:");
    Serial.println("[CO]     o23 10 10 01 73 61 76 65   写 0x1010:01 = \"save\" 存参数");
    Serial.println("[CO]     o23 11 10 01 6C 6F 61 64   写 0x1011:01 = \"load\" 恢复出厂");
    Serial.println("[CO]     o2B 00 18 05 14 00         写 0x1800:05 TPDO 周期 = 20ms");
    Serial.println("[CO] 自定义帧（空格分隔的十六进制字节）:");
    Serial.println("[CO]   o40 00 10 00         读 0x1000 设备类型");
    Serial.println("[CO]   o2B 17 10 00 C8 00   写 0x1017 = 200ms");
    Serial.println("[CO]   o2 40 64 60 00      读节点2 的 0x6064");
    return;
  }

  // ★ SDO 报文按 CiA 301 固定 8 字节，没用到的字节必须补 0。
  //   之前只发 4 字节，从站判定"帧太短"直接 abort（错误码 0x05040001）。
  for (uint8_t i = n; i < 8; i++) d[i] = 0;

  if (!foc_can_set_loopback(true)) {
    Serial.println("[CO] 切回环模式失败（CAN 没起来？）");
    return;
  }
  delay(5);

  Serial.printf("[CO] === 手动主站：请求发往 0x%03X（节点%u %s）===\n",
                0x600u + node, (unsigned)node, node == 1 ? "M0 yaw" : "M1 pitch");
  co_trace = true;

  uint8_t rsp[8];
  memset(rsp, 0, sizeof(rsp));
  bool ok = co_xfer(0x600u + node, d, 8, 0x580u + node, rsp, 300);

  if (!ok) {
    Serial.printf("[CO] 300ms 内无 0x%03X 应答（字节没组成合法 SDO 请求，或被从站中止）\n",
                  0x580u + node);
  }
  co_trace = false;
  foc_can_set_loopback(false);
  Serial.println("[CO] === 结束，切回正常模式 ===");
}
