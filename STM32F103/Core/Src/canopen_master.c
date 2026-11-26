/**
  ******************************************************************************
  * @file    canopen_master.c
  * @brief   F103 当 CANopen 主站（最小实现）。
  *
  * 这里只做四件事，每件都是真实伺服手册里主站必须干的：
  *   1) SDO 读写：0x600+节点 发请求，0x580+节点 收应答
  *   2) NMT：0x000 发 01 <节点> 把从站启动到 Operational
  *   3) CiA 402 四档：0x6040 依次写 06 / 07 / 0F
  *   4) RPDO：0x200+节点 每 20ms 发一帧（控制字 2 字节 + 目标位置 4 字节）
  *
  * 全部用轮询，不用中断：发一帧等一帧，逻辑线性，好调试。
  *
  * 编码约定（很重要，改代码前先看这段）：
  *   本文件必须存成 GB2312/ANSI。ARMCC V5 只按本地代码页读源文件，
  *   文件若是 UTF-8，中文会被拆成乱码，末尾字节还会吃掉引号，
  *   报 missing closing quote 这种看不懂的错。
  *   另外：串口打印的字符串一律用英文。因为终端若是 UTF-8，
  *   GB2312 的中文会显示成乱码；英文在任何编码下都不会错。
  ******************************************************************************
  */

#include "canopen_master.h"
#include "can.h"
#include "usart.h"

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>

/* ---------------- 可调参数 ---------------- */
#define COM_SDO_TIMEOUT    100u      /* 等 SDO 应答最长 100ms */
#define COM_CSP_PERIOD     20u       /* CSP 周期 20ms（和 ESP32 那边一致） */
#define COM_SINE_AMP       1500L     /* 正弦幅度 ±15.00°（单位 0.01°） */
#define COM_SINE_PERIOD    2000u     /* 正弦一圈 2 秒 */
#define COM_SINE_DURATION  20000u    /* 跑 20 秒自动停 */
#define COM_CIRC_RADIUS    3000L     /* 画圆半径 ±30°（单位 0.01°） */
#define COM_CIRC_PERIOD    8000u     /* 一圈 8 秒 */
#define COM_EIGHT_AX       3500L     /* 8 字 yaw 半幅 ±35° */
#define COM_EIGHT_AY       2500L     /* 8 字 pitch 半幅 ±25° */
#define COM_EIGHT_PERIOD   10000u    /* 一个完整 8 字 10 秒 */
#define COM_SPIRAL_RADIUS  3000L     /* 螺旋最大半径 ±30° */
#define COM_SPIRAL_TURNS   3.0f      /* 由内到外再收回，共转 3 圈 */
#define COM_SPIRAL_PERIOD  15000u    /* 整段螺旋 15 秒，走完自动停 */

/* ---------------- 运行状态 ---------------- */
static uint8_t  s_node     = 1;      /* 当前操作哪个从站：1=M0(yaw) 2=M1(pitch) */
static uint8_t  s_enabled  = 0;      /* 是否已经在 Operation enabled */
static int32_t  s_target   = 0;      /* 当前目标位置，单位 0.01° */
static int32_t  s_pos_now  = 0;      /* 最近一次读到的实际位置 */

static uint8_t  s_dual     = 0;      /* 1 = dual mode: +/- drives BOTH axes */
static int32_t  s_target2  = 0;      /* node 2 target (0.01 deg), used in dual mode */
static uint8_t  s_sine_on  = 0;      /* 连续正弦是否在跑 */
static uint32_t s_sine_t0  = 0;      /* 正弦起始时刻 */
static uint32_t s_sine_last= 0;      /* 上一拍发送时刻 */
static int32_t  s_sine_c   = 0;      /* 正弦中心（起跑时取当前位置） */
static uint8_t  s_traj     = 0;      /* 轨迹类型：0=停 1=圆 2=8 字 3=螺旋 */
static uint32_t s_traj_t0  = 0;      /* 轨迹起始时刻 */
static uint32_t s_traj_last= 0;      /* 轨迹上一拍发送时刻 */

/* 轨迹类型（s_traj 取值） */
#define TRAJ_OFF     0u
#define TRAJ_CIRCLE  1u
#define TRAJ_EIGHT   2u
#define TRAJ_SPIRAL  3u

static char     s_buf[160];

/* ============================================================
 * 基础：打印
 * ============================================================ */
static void com_log(const char *fmt, ...)
{
    va_list ap;
    int n;

    va_start(ap, fmt);
    n = vsnprintf(s_buf, sizeof(s_buf), fmt, ap);
    va_end(ap);

    if (n > 0) {
        if (n > (int)sizeof(s_buf) - 1) n = (int)sizeof(s_buf) - 1;
        HAL_UART_Transmit(&huart1, (uint8_t *)s_buf, (uint16_t)n, 200);
    }
}

/* ============================================================
 * 基础：发一帧 / 收一帧
 * ============================================================ */
static uint8_t com_tx(uint32_t id, const uint8_t *d, uint8_t len)
{
    CAN_TxHeaderTypeDef h;
    uint32_t mailbox;

    h.StdId              = id;
    h.ExtId              = 0;
    h.IDE                = CAN_ID_STD;
    h.RTR                = CAN_RTR_DATA;
    h.DLC                = len;
    h.TransmitGlobalTime = DISABLE;

    return (HAL_CAN_AddTxMessage(&hcan, &h, (uint8_t *)d, &mailbox) == HAL_OK) ? 1 : 0;
}

/* 非阻塞取一帧，取到返回 1 */
static uint8_t com_rx(CAN_RxHeaderTypeDef *rh, uint8_t *d)
{
    if (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) == 0) return 0;

    /* 用 HAL_CAN_GetRxMessage 之前必须确认 FIFO 非空，否则会返回错误并丢掉 */
    if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, rh, d) != HAL_OK) return 0;
    return 1;
}

/**
  * @brief 等一个指定 ID 的帧。等不到返回 0。
  *        等待期间收到的其它帧（TPDO 0x180+、EMCY 0x080+）会被跳过或提示，
  *        这是必须的 —— 从站每 20ms 主动发一帧 TPDO，不能把它当超时。
  */
static uint8_t com_wait(uint32_t id, uint8_t *out, uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    CAN_RxHeaderTypeDef rh;
    uint8_t buf[8];

    while ((HAL_GetTick() - t0) < timeout_ms) {
        if (!com_rx(&rh, buf)) continue;

        if (rh.StdId == id) {
            memcpy(out, buf, 8);
            return 1;
        }
        if (rh.StdId == 0x081u || rh.StdId == 0x082u) {
            com_log("  [EMCY] slave alarm: code 0x%02X%02X\n", buf[1], buf[0]);
        }
    }
    return 0;
}

/* ============================================================
 * SDO 读 / 写
 *   请求帧固定 8 字节：40=读  2B=写2字节  23=写4字节
 *   应答帧 0x580+node：43/4B 里带 4/2 字节数据，80 表示中止(abort)
 * ============================================================ */
static uint8_t com_sdo_read_n(uint8_t node, uint16_t idx, uint8_t sub, uint32_t *val)
{
    uint8_t req[8];
    uint8_t rsp[8];

    req[0] = 0x40;
    req[1] = (uint8_t)(idx & 0xFF);
    req[2] = (uint8_t)(idx >> 8);
    req[3] = sub;
    req[4] = 0; req[5] = 0; req[6] = 0; req[7] = 0;

    if (!com_tx(0x600u + node, req, 8)) {
        com_log("  TX fail: CAN mailbox full (bus wiring ?)\n");
        return 0;
    }
    if (!com_wait(0x580u + node, rsp, COM_SDO_TIMEOUT)) {
        com_log("  no reply from 0x%03X in %u ms\n", 0x580u + node, COM_SDO_TIMEOUT);
        com_log("  check: CAN_H/L swapped ? | two 120R ? | common GND ? | both 500k ?\n");
        return 0;
    }
    if (rsp[0] == 0x80) {
        uint32_t code = (uint32_t)rsp[4] | ((uint32_t)rsp[5] << 8)
                      | ((uint32_t)rsp[6] << 16) | ((uint32_t)rsp[7] << 24);
        com_log("  slave ABORT, code 0x%08lX (object/state not allowed)\n",
                (unsigned long)code);
        return 0;
    }

    *val = (uint32_t)rsp[4] | ((uint32_t)rsp[5] << 8)
         | ((uint32_t)rsp[6] << 16) | ((uint32_t)rsp[7] << 24);
    return 1;
}

/* 第 11 步：老接口固定操作 s_node；双轴要指定节点，所以加一层包装 */
static uint8_t com_sdo_read(uint16_t idx, uint8_t sub, uint32_t *val)
{
    return com_sdo_read_n(s_node, idx, sub, val);
}

static uint8_t com_sdo_write16_n(uint8_t node, uint16_t idx, uint8_t sub, uint16_t v)
{
    uint8_t req[8];
    uint8_t rsp[8];

    req[0] = 0x2B;
    req[1] = (uint8_t)(idx & 0xFF);
    req[2] = (uint8_t)(idx >> 8);
    req[3] = sub;
    req[4] = (uint8_t)(v & 0xFF);
    req[5] = (uint8_t)(v >> 8);
    req[6] = 0; req[7] = 0;

    if (!com_tx(0x600u + node, req, 8)) return 0;
    if (!com_wait(0x580u + node, rsp, COM_SDO_TIMEOUT)) return 0;
    return (rsp[0] == 0x60) ? 1 : 0;
}

static uint8_t com_sdo_write16(uint16_t idx, uint8_t sub, uint16_t v)
{
    return com_sdo_write16_n(s_node, idx, sub, v);
}

/* ============================================================
 * NMT / CiA 402
 * ============================================================ */
static void com_nmt_start(uint8_t node)
{
    uint8_t d[2];
    d[0] = 0x01;        /* 1 = Start Remote Node */
    d[1] = node;
    com_tx(0x000u, d, 2);
}

/* 状态字 → 人话（掩码 0x006F，低 7 位才是状态，高位是标志） */
static const char *com_st_name(uint32_t sw)
{
    uint16_t s = (uint16_t)(sw & 0x006Fu);

    if (s == 0x0008) return "Fault";
    if (s == 0x0040) return "Switch on disabled";
    if (s == 0x0021) return "Ready to switch on";
    if (s == 0x0023) return "Switched on";
    if (s == 0x0027) return "Operation enabled";
    return "Unknown";
}

/* 发一个控制字，然后读状态字确认 */
static void com_cia_step_n(uint8_t node, uint16_t cw)
{
    uint32_t sw = 0;

    if (!com_sdo_write16_n(node, 0x6040, 0, cw)) {
        com_log("  node %u: write 0x6040 = 0x%04X failed\n", (unsigned)node, cw);
        return;
    }
    HAL_Delay(10);
    if (com_sdo_read_n(node, 0x6041, 0, &sw)) {
        com_log("  node %u: 0x6040 = 0x%04X  ->  status 0x%04X  %s\n",
                (unsigned)node, cw, (unsigned)sw, com_st_name(sw));
    }
}

static void com_cia_step(uint16_t cw)
{
    com_cia_step_n(s_node, cw);
}

/* ============================================================
 * 故障复位（第 12 步补）
 *   CiA 402 最硬的一条：Fault 是锁存态，只有 0x6040 的 bit7 上升沿能出来。
 *   所以必须"先让 bit7 归零（发 0x0000），再发 0x0080"，这才有沿。
 *   直接发 0x0080（前面正好也是 0x0080）是没有沿的，什么都不会发生；
 *   发 0x0000 也不过是在 Fault 里被忽略 —— 这两点踩过坑。
 * ============================================================ */
static void com_fault_reset(uint8_t node)
{
    uint32_t sw = 0;

    if (com_sdo_read_n(node, 0x6041, 0, &sw) && ((sw & 0x006Fu) == 0x0008))
        com_log("  node %u was in FAULT, resetting ...\r\n", (unsigned)node);

    com_sdo_write16_n(node, 0x6040, 0, 0x0000);   /* bit7 = 0 */
    HAL_Delay(10);
    com_sdo_write16_n(node, 0x6040, 0, 0x0080);   /* bit7 上升沿 -> 复位 */
    HAL_Delay(10);

    if (com_sdo_read_n(node, 0x6041, 0, &sw))
        com_log("  node %u status 0x%04X  %s\r\n",
                (unsigned)node, (unsigned)sw, com_st_name(sw));
}

/* ============================================================
 * RPDO：0x200+节点，6 字节 = 控制字(2) + 目标位置(4)，小端
 * ============================================================ */
static void com_rpdo_pos_to(uint8_t node, uint16_t cw, int32_t pos100)
{
    uint8_t d[6];

    d[0] = (uint8_t)(cw & 0xFF);
    d[1] = (uint8_t)(cw >> 8);
    d[2] = (uint8_t)(pos100 & 0xFF);
    d[3] = (uint8_t)((pos100 >> 8) & 0xFF);
    d[4] = (uint8_t)((pos100 >> 16) & 0xFF);
    d[5] = (uint8_t)((pos100 >> 24) & 0xFF);

    com_tx(0x200u + node, d, 6);
}

static void com_rpdo_pos(uint16_t cw, int32_t pos100)
{
    com_rpdo_pos_to(s_node, cw, pos100);
}

/* ============================================================
 * 串口命令
 * ============================================================ */
static void com_help(void)
{
    com_log("\r\n===== F103 CANopen master =====\r\n");
    com_log("  t   selftest: SDO read 0x1000 (run this first)\r\n");
    com_log("  r   read current position 0x6064\r\n");
    com_log("  e   enable (NMT + 06 / 07 / 0F)\r\n");
    com_log("  x   disable (0x6040 = 0x0000)\r\n");
    com_log("  +   target +10 deg      -   target -10 deg\r\n");
    com_log("  s   sine follow on/off (+/-15 deg, 2 s/cycle, 20 s)\r\n");
    com_log("  m   monitor: listen 5 s, print every frame (RX path check)\r\n");
    com_log("  f   fault reset (0x6040 bit7 rising edge), then press e\r\n");
    com_log("  b   dual: enable BOTH axes, then +/- moves yaw+pitch together\r\n");
    com_log("  0   both axes to 0 deg      9   both to 90 deg\r\n");
    com_log("  8   both axes to 180 deg\r\n");
    com_log("  a   absolute: a<yaw> <pitch>, end with '='  e.g. a45 90=  (= or Enter)\r\n");
    com_log("  g   point:    g<azimuth> <elevation>=   e.g. g30 15=   (0 0 = ahead, level)\r\n");
    com_log("  p   aim at XYZ: p<x> <y> <z>=   e.g. p100 200 50=   (y ahead, x right, z up)\r\n");
    com_log("  c   circle:   r=30 deg, 8 s/rev, press c to stop\r\n");
    com_log("  i   figure-8: yaw +-35, pitch +-25 deg, 10 s/loop, press i to stop\r\n");
    com_log("  w   spiral:   3 turns 0->30->0 deg, 15 s, auto stop\r\n");
    com_log("  1   node 1 (M0 yaw)     2   node 2 (M1 pitch)\r\n");
    com_log("===============================\r\n");
}

/* monitor: 5 s ---------
 * Listen 5 s and print every frame on the bus.
 * Diagnosis: frames coming in = RX path fine, so the fault is elsewhere.
 *            zero frames        = RX path dead (PA11 / RXD / transceiver). */
static void com_cmd_monitor(void)
{
    uint32_t t0 = HAL_GetTick();
    CAN_RxHeaderTypeDef rh;
    uint8_t buf[8];
    unsigned n = 0;

    com_log("\r\n[monitor] listening 5 s ...\r\n");
    while ((HAL_GetTick() - t0) < 5000) {
        if (!com_rx(&rh, buf)) continue;
        n++;
        com_log("  0x%03lX [%lu] %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
                (unsigned long)rh.StdId, (unsigned long)rh.DLC,
                buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
        if (n >= 40) break;
    }
    com_log("[monitor] %u frame(s) in 5 s\r\n", n);
    if (n == 0) com_log("  -> RX path DEAD: check TJA1050 RXD to PA11, 5V supply, pin S to GND\r\n");
    else        com_log("  -> RX path OK (frames are arriving)\r\n");
}

static void com_cmd_selftest(void)
{
    uint32_t v = 0;

    com_log("\r\n[TEST] SDO read node %u 0x1000 device type ...\r\n", s_node);
    if (com_sdo_read(0x1000, 0, &v)) {
        com_log("  got 0x%08lX  -> BUS OK (request and reply both fine)\r\n", (unsigned long)v);
    } else {
        com_log("  FAILED. check hardware first, not code:\r\n");
        com_log("  1) power off, measure CAN_H-CAN_L: should be 60 ohm\r\n");
        com_log("  2) TJA1050 VCC = 5V ? pin S tied to GND ?\r\n");
        com_log("  3) TXD->PA12, RXD->PA11 (same name to same name)\r\n");
        com_log("  4) all boards share GND ?\r\n");
    }
}

static void com_cmd_readpos(void)
{
    uint32_t v = 0;

    if (com_sdo_read(0x6064, 0, &v)) {
        s_pos_now = (int32_t)v;
        com_log("node%u pos = %ld.%02ld deg  (0x6064 = %ld)\r\n",
                s_node,
                (long)(s_pos_now / 100), (long)(s_pos_now < 0 ? (-s_pos_now % 100) : (s_pos_now % 100)),
                (long)s_pos_now);
    }
}

static void com_cmd_enable(void)
{
    uint32_t sw = 0;
    uint32_t v  = 0;

    com_log("\r\n[ENABLE] node %u\r\n", s_node);

    /* 1) NMT：让从站进 Operational，否则 PDO 不工作 */
    com_nmt_start(s_node);
    HAL_Delay(30);

    /* 2) 先复位故障：Fault 里只认 bit7 上升沿，直接发 06 是出不来的 */
    com_fault_reset(s_node);

    /* 3) 三步推档。不能一步到 0x0F —— 从站会拒绝非法跳转 */
    com_cia_step(0x0006);       /* Shutdown      -> Ready to switch on */
    com_cia_step(0x0007);       /* Switch on     -> Switched on */
    com_cia_step(0x000F);       /* Enable op     -> Operation enabled */

    /* 3) 使能后立刻把 目标 = 当前位置，否则从站会把轴拽向旧目标 */
    if (com_sdo_read(0x6041, 0, &sw) && ((sw & 0x006Fu) == 0x0027)) {
        s_enabled = 1;
        s_dual    = 0;
        if (com_sdo_read(0x6064, 0, &v)) {
            s_target  = (int32_t)v;
            s_pos_now = s_target;
            com_log("  enabled, target locked at %ld.%02ld deg\r\n",
                    (long)(s_target / 100),
                    (long)(s_target < 0 ? (-s_target % 100) : (s_target % 100)));
        }
    } else {
        com_log("  not in Operation enabled, see which step was rejected\r\n");
    }
}

/* ============================================================
 * 第 11 步：双轴
 *   b  -> 两个节点各自走一遍 06 / 07 / 0F，成功后 +/- 同时驱动两轴
 *   1/2 -> 回到单轴（老行为不变）
 * ============================================================ */
static int32_t com_clamp(int32_t v100)
{
    if (v100 >  18000L) return  18000L;
    if (v100 < -18000L) return -18000L;
    return v100;
}

static void com_log_deg(const char *tag, int32_t v100)
{
    long a = (long)(v100 < 0 ? -v100 : v100);

    com_log("   %s %s%ld.%02ld deg\r\n", tag,
            (v100 < 0 ? "-" : ""), a / 100, a % 100);
}

static void com_cmd_enable_both(void)
{
    uint32_t v1 = 0;
    uint32_t v2 = 0;
    uint32_t sw = 0;
    uint8_t  ok = 1;
    uint8_t  n;

    com_log("\r\n[DUAL] enable node 1 (yaw) + node 2 (pitch)\r\n");

    /* 1) 两个节点都进 Operational，否则 PDO 不生效 */
    com_nmt_start(1);
    com_nmt_start(2);
    HAL_Delay(30);

    /* 2) 一个节点一个节点：先复位故障，再推三步 */
    for (n = 1; n <= 2; n++) {
        com_fault_reset(n);
        com_cia_step_n(n, 0x0006);
        com_cia_step_n(n, 0x0007);
        com_cia_step_n(n, 0x000F);
        if (!com_sdo_read_n(n, 0x6041, 0, &sw) || ((sw & 0x006Fu) != 0x0027)) ok = 0;
    }
    if (!ok) {
        com_log("  DUAL enable FAILED: see which node did not reach Operation enabled\r\n");
        s_dual    = 0;
        s_enabled = 0;
        return;
    }

    /* 3) 目标先锁在各自当前位置，别让上电姿态被拽走 */
    com_sdo_read_n(1, 0x6064, 0, &v1);
    com_sdo_read_n(2, 0x6064, 0, &v2);
    s_target  = (int32_t)v1;
    s_target2 = (int32_t)v2;

    s_enabled = 1;
    s_dual    = 1;
    s_sine_on = 0;

    com_log("  DUAL mode ON:  +/- moves BOTH axes\r\n");
    com_log_deg("yaw  ", s_target);
    com_log_deg("pitch", s_target2);
    com_log("  tip: press 1 or 2 to go back to a single axis\r\n");
}

static void com_cmd_disable(void)
{
    s_sine_on = 0;
    s_traj = 0;
    com_cia_step(0x0000);       /* Disable voltage -> Switch on disabled */
    s_enabled = 0;
    s_dual    = 0;
}

static void com_cmd_jog(int32_t delta)
{
    if (!s_enabled) {
        com_log("  not enabled yet, press e first\r\n");
        return;
    }
    if (s_dual) {
        long ay;
        long ap;

        s_target  = com_clamp(s_target  + delta);
        s_target2 = com_clamp(s_target2 + delta);

        /* 两帧连着发：0x201 给 yaw，0x202 给 pitch */
        com_rpdo_pos_to(1, 0x000F, s_target);
        com_rpdo_pos_to(2, 0x000F, s_target2);

        ay = (long)(s_target  < 0 ? -s_target  : s_target);
        ap = (long)(s_target2 < 0 ? -s_target2 : s_target2);
        com_log("  DUAL 0x201+0x202  yaw %s%ld.%02ld  pitch %s%ld.%02ld deg\r\n",
                (s_target  < 0 ? "-" : ""), ay / 100, ay % 100,
                (s_target2 < 0 ? "-" : ""), ap / 100, ap % 100);
        return;
    }

    s_target = com_clamp(s_target + delta);
    com_rpdo_pos(0x000F, s_target);
    com_log("  target %ld.%02ld deg (PDO 0x%03X, 6 bytes)\r\n",
            (long)(s_target / 100),
            (long)(s_target < 0 ? (-s_target % 100) : (s_target % 100)),
            0x200u + s_node);
}

/* ============================================================
 * a<yaw> <pitch>  绝对定位（一次到位，不用按十几下 +/-）
 *   串口工具只能发单字符，所以这里自己攒一行：按 a 之后收到的字符
 *   都进缓冲区，回车才解析。其它单字符命令完全不受影响。
 * ============================================================ */
static char     s_line[24];
static uint8_t  s_line_len = 0;
static uint8_t  s_line_on  = 0;
static uint32_t s_line_ts  = 0;
static uint8_t  s_line_kind = 0;     /* 0 = a(绝对角度)   1 = g(指向) */      /* 最后一个字符到达的时刻，用于无回车自动提交 */

/* 读一个十进制数（可带负号、小数点），*pp 会前移到数字之后 */
static float com_parse_float(const char **pp, int *ok)
{
    const char *p = *pp;
    float v = 0.0f, frac = 0.1f;
    int   neg = 0, got = 0;

    while (*p == ' ' || *p == ',') p++;
    if (*p == '-') { neg = 1; p++; }
    else if (*p == '+') p++;

    while (*p >= '0' && *p <= '9') { v = v * 10.0f + (float)(*p - '0'); p++; got = 1; }
    if (*p == '.') {
        p++;
        while (*p >= '0' && *p <= '9') {
            v += (float)(*p - '0') * frac;
            frac *= 0.1f;
            got = 1;
            p++;
        }
    }

    *pp = p;
    if (ok) *ok = got;
    return neg ? -v : v;
}

/* 两轴直接去指定角度（单位 0.01 度） */
static void com_cmd_goto(int32_t y100, int32_t p100)
{
    if (!s_dual) {
        com_log("  press b first (dual mode on)\r\n");
        return;
    }

    s_target  = com_clamp(y100);
    s_target2 = com_clamp(p100);

    com_rpdo_pos_to(1, 0x000F, s_target);
    com_rpdo_pos_to(2, 0x000F, s_target2);

    com_log("  ABS 0x201+0x202\r\n");
    com_log_deg("yaw  ", s_target);
    com_log_deg("pitch", s_target2);
}

static void com_cmd_abs(void)
{
    const char *p = s_line;
    float ay, ap;
    int   ok_y = 0, ok_p = 0;

    if (!s_dual) {
        com_log("  press b first (dual mode on), then a<yaw> <pitch>\r\n");
        return;
    }

    ay = com_parse_float(&p, &ok_y);
    ap = com_parse_float(&p, &ok_p);

    if (!ok_y) {                    /* 空输入：什么都不发，别让两轴跑回 0 */
        com_log("  no number typed, nothing sent\r\n");
        return;
    }
    if (!ok_p) {                    /* 只给了 yaw：pitch 保持原目标不动 */
        ap = (float)s_target2 / 100.0f;
    }

    com_cmd_goto((int32_t)(ay * 100.0f), (int32_t)(ap * 100.0f));
}

/* ============================================================
 * g<方位角> <仰角>   指向命令（人话版，不用自己算 yaw/pitch）
 *   方位角 azimuth : 0 = 正前方，+ / - 各 90 度（yaw 只能在 0~180 内）
 *   仰角 elevation : 0 = 水平，+ 抬头、- 低头（±90）
 *   换算（实测标定）：
 *     pitch = 90 + 仰角     （0 朝地 / 90 朝前 / 180 朝天）
 *     yaw   = 90 + 方位角    （yaw 90 = 正前方）
 * ============================================================ */
#define COM_POINT_YAW_SIGN (+1.0f)   /* 实测方位反了就改 -1.0f */

static void com_cmd_point(void)
{
    const char *p = s_line;
    float az, el;
    int   ok_az = 0, ok_el = 0;

    if (!s_dual) {
        com_log("  press b first (dual mode on)\r\n");
        return;
    }

    az = com_parse_float(&p, &ok_az);
    el = com_parse_float(&p, &ok_el);

    if (!ok_az) {
        com_log("  no number typed, nothing sent\r\n");
        return;
    }
    if (!ok_el) el = 0.0f;          /* 只给方位角：默认仰角 0（水平） */

    if (az < -90.0f) az = -90.0f;
    if (az >  90.0f) az =  90.0f;
    if (el < -90.0f) el = -90.0f;
    if (el >  90.0f) el =  90.0f;

    com_log("  POINT az %d  el %d deg\r\n", (int)az, (int)el);

    com_cmd_goto((int32_t)((90.0f + az * COM_POINT_YAW_SIGN) * 100.0f),
                 (int32_t)((90.0f + el) * 100.0f));
}

/* ============================================================
 * p<x> <y> <z>   指向空间中的一点（笛卡尔坐标，单位随意：米 / 厘米都行）
 *   坐标系（原点在云台，跟着云台机身走）：
 *     y = 正前方        x = 右方        z = 上方
 *   算法：
 *     方位角 = atan2(x, y)                    x>0 在右侧 → 方位正向是顺时针
 *     仰角   = atan2(z, sqrt(x*x + y*y))      水平距离越远，同样高度仰角越小
 *   再按 g 命令的换算交给 com_cmd_goto：yaw = 90 + 方位角, pitch = 90 + 仰角
 * ============================================================ */
static void com_cmd_xyz(void)
{
    const char *p = s_line;
    float x, y, z, r, az, el;
    int   ok_x = 0, ok_y = 0, ok_z = 0;

    if (!s_dual) {
        com_log("  press b first (dual mode on)\r\n");
        return;
    }

    x = com_parse_float(&p, &ok_x);
    y = com_parse_float(&p, &ok_y);
    z = com_parse_float(&p, &ok_z);

    if (!ok_x || !ok_y) {
        com_log("  need at least x and y, e.g.  100 200 50=\r\n");
        return;
    }
    if (!ok_z) z = 0.0f;

    r = sqrtf(x * x + y * y);
    if (r < 0.0001f && fabsf(z) < 0.0001f) {
        com_log("  that is the origin itself, ignored\r\n");
        return;
    }

    az = atan2f(x, y) * 57.2957795f;
    el = atan2f(z, r) * 57.2957795f;

    com_log("  XYZ %d,%d,%d -> az %d el %d\r\n",
            (int)x, (int)y, (int)z, (int)az, (int)el);

    if (az > 90.0f || az < -90.0f) {
        com_log("  note: azimuth beyond +-90, will hit yaw limit\r\n");
    }

    com_cmd_goto((int32_t)((90.0f + az) * 100.0f),
                 (int32_t)((90.0f + el) * 100.0f));
}

/* 提交缓冲区：回车和"= "两条路都走这里 */
static void com_line_submit(void)
{
    s_line[s_line_len] = 0;
    s_line_on  = 0;
    s_line_len = 0;

    if      (s_line_kind == 1) com_cmd_point();
    else if (s_line_kind == 2) com_cmd_xyz();
    else                       com_cmd_abs();   /* 读 s_line，内容仍在 */
}

/* 轨迹：圆 / 8 字 / 螺旋，共用一台状态机。中心都在正前方 (90, 90)。
 * 同一个键再按一次 = 停；螺旋走满一圈后自己停。 */
static void com_cmd_traj(uint8_t mode)
{
    if (s_traj == mode) {                 /* 同一个键再按一次 -> 停 */
        s_traj = TRAJ_OFF;
        com_log("trajectory stopped\r\n");
        return;
    }
    if (!s_dual) {
        com_log("  press b first (dual mode on)\r\n");
        return;
    }

    s_sine_on   = 0;
    s_traj      = mode;
    s_traj_t0   = HAL_GetTick();
    s_traj_last = s_traj_t0;

    if (mode == TRAJ_CIRCLE) {
        com_log("circle start: r=30 deg, 8 s/rev, press c again to stop\r\n");
    } else if (mode == TRAJ_EIGHT) {
        com_log("figure-8 start: yaw +-35, pitch +-25 deg, 10 s/loop, press i again to stop\r\n");
    } else {
        com_log("spiral start: 3 turns 0 -> 30 -> 0 deg, 15 s, auto stop\r\n");
    }
}

static void com_cmd_sine(void)
{
    if (s_sine_on) {
        s_sine_on = 0;
        com_log("sine follow stopped\r\n");
        return;
    }
    if (!s_enabled) {
        com_log("  not enabled yet, press e first\r\n");
        return;
    }
    s_traj      = 0;
    s_sine_c    = s_pos_now;        /* 以当前位置为中心，避免起跑台阶 */
    s_sine_t0   = HAL_GetTick();
    s_sine_last = s_sine_t0;
    s_sine_on   = 1;
    com_log("sine follow start: +/-15 deg, 2 s/cycle, 20 s\r\n");
}

static void com_cmd(uint8_t c)
{
    /* a 之后进入"收参数"状态：攒一行，回车提交，退格可改 */
    if (s_line_on) {
        if (c == '\r' || c == '\n' || c == '=') {   /* 回车或 = ：立刻发 */
            com_line_submit();
            return;
        }
        if (c == 8 || c == 127) {           /* 退格 */
            if (s_line_len) s_line_len--;
            com_log("\b \b");
            s_line_ts = HAL_GetTick();
            return;
        }
        if (s_line_len < (uint8_t)(sizeof(s_line) - 1)) {
            s_line[s_line_len++] = (char)c;
            com_log("%c", c);               /* 回显，不然看不见自己敲了啥 */
        }
        s_line_ts = HAL_GetTick();
        return;
    }

    switch (c) {
    case 'h': case 'H': com_help();          break;
    case 't': case 'T': com_cmd_selftest();  break;
    case 'r': case 'R': com_cmd_readpos();   break;
    case 'e': case 'E': com_cmd_enable();    break;
    case 'x': case 'X': com_cmd_disable();   break;
    case '+':           com_cmd_jog(+1000);  break;
    case '-':           com_cmd_jog(-1000);  break;
    case 's': case 'S': com_cmd_sine();      break;
    case 'm': case 'M': com_cmd_monitor();   break;
    case 'f': case 'F':
              com_log("\r\n[RESET] node %u fault reset\r\n", s_node);
              com_fault_reset(s_node);
              break;
    case 'c': case 'C': com_cmd_traj(TRAJ_CIRCLE); break;
    case 'i': case 'I': com_cmd_traj(TRAJ_EIGHT);  break;
    case 'w': case 'W': com_cmd_traj(TRAJ_SPIRAL); break;
    case 'b': case 'B': com_cmd_enable_both(); break;
    case 'a': case 'A':
              s_line_on   = 1;
              s_line_len  = 0;
              s_line_kind = 0;
              s_line_ts   = HAL_GetTick();
              com_log("  type yaw + space + pitch, e.g.  90 90=   (or Enter)\r\n");
              break;
    case 'p': case 'P':
              s_line_on   = 1;
              s_line_len  = 0;
              s_line_kind = 2;
              s_line_ts   = HAL_GetTick();
              com_log("  type x + space + y + space + z, e.g.  100 200 50=   (y=ahead x=right z=up)\r\n");
              break;
    case 'g': case 'G':
              s_line_on   = 1;
              s_line_len  = 0;
              s_line_kind = 1;
              s_line_ts   = HAL_GetTick();
              com_log("  type azimuth + space + elevation, e.g.  30 15=   (0 0 = ahead level)\r\n");
              break;
    case '0': com_cmd_goto(0, 0);             break;
    case '9': com_cmd_goto(9000, 9000);       break;
    case '8': com_cmd_goto(18000, 18000);     break;
    case '1': s_node = 1; s_enabled = 0; s_sine_on = 0; s_traj = 0; s_dual = 0;
              com_log("switch to node 1 (M0 yaw)\r\n"); break;
    case '2': s_node = 2; s_enabled = 0; s_sine_on = 0; s_traj = 0; s_dual = 0;
              com_log("switch to node 2 (M1 pitch)\r\n"); break;
    default: break;
    }
}

/* ============================================================
 * 对外接口
 * ============================================================ */
/* ============================================================
 * 串口接收：中断 + 环形缓冲
 *   F103 的 USART1 没有接收 FIFO，一次发 "90 90=" 是 6 字节、
 *   每 87us 到达一个，主循环轮询只要慢一拍就覆盖丢字节。
 * ============================================================ */
static uint8_t          s_rx_byte;
static uint8_t          s_rx_ring[64];
static volatile uint8_t s_rx_head = 0;
static volatile uint8_t s_rx_tail = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t n;

    if (huart->Instance != USART1) return;

    n = (uint8_t)((s_rx_head + 1u) & 63u);
    if (n != s_rx_tail) {              /* 缓冲满就丢弃新字节，不覆盖旧数据 */
        s_rx_ring[s_rx_head] = s_rx_byte;
        s_rx_head = n;
    }
    HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1);
}

/* 溢出(ORE)/帧错误：必须重启接收，否则 HAL 把 RxState 卡在 BUSY_RX 再也不收 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1) return;

    __HAL_UART_CLEAR_OREFLAG(huart);
    huart->RxState = HAL_UART_STATE_READY;
    HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1);
}

void com_init(void)
{
    CAN_FilterTypeDef f;

    /* 过滤器全放行：标准帧、掩码全 0 → 任何 ID 都进 FIFO0 */
    f.FilterBank           = 0;
    f.FilterMode           = CAN_FILTERMODE_IDMASK;
    f.FilterScale          = CAN_FILTERSCALE_32BIT;
    f.FilterIdHigh         = 0x0000;
    f.FilterIdLow          = 0x0000;
    f.FilterMaskIdHigh     = 0x0000;
    f.FilterMaskIdLow      = 0x0000;
    f.FilterFIFOAssignment = CAN_RX_FIFO0;
    f.FilterActivation     = ENABLE;
    f.SlaveStartFilterBank = 14;

    /* 串口中断最先开：即使下面 CAN 初始化失败，调试通道也得活着 */
    HAL_NVIC_SetPriority(USART1_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1);

    if (HAL_CAN_ConfigFilter(&hcan, &f) != HAL_OK) {
        com_log("[CAN] filter config failed\r\n");
        return;
    }
    if (HAL_CAN_Start(&hcan) != HAL_OK) {
        com_log("[CAN] start failed (check HSE crystal)\r\n");
        return;
    }

    com_log("\r\n[CAN] master ready: 500kbps, PA12=TX PA11=RX\r\n");
    com_log("[CAN] press h for help\r\n");
}

void com_poll(void)
{
    /* ---- 串口：字符已由中断存进环形缓冲，这里取出来逐字处理 ---- */
    while (s_rx_tail != s_rx_head) {
        uint8_t c = s_rx_ring[s_rx_tail];

        s_rx_tail = (uint8_t)((s_rx_tail + 1u) & 63u);
        com_cmd(c);
    }

    /* ---- 轨迹（圆 / 8 字 / 螺旋）：每 20ms 发一对 RPDO ---- */
    if (s_traj) {
        uint32_t now = HAL_GetTick();

        if ((now - s_traj_last) >= COM_CSP_PERIOD) {
            uint32_t el;
            float    th, r;
            int32_t  cy, cp;

            s_traj_last = now;
            el = now - s_traj_t0;

            if (s_traj == TRAJ_CIRCLE) {
                th = 6.2831853f * (float)el / (float)COM_CIRC_PERIOD;
                cy = 9000L + (int32_t)((float)COM_CIRC_RADIUS * sinf(th));
                cp = 9000L + (int32_t)((float)COM_CIRC_RADIUS * cosf(th));
            } else if (s_traj == TRAJ_EIGHT) {
                /* 8 字：yaw 走 1 个周期、pitch 走 2 个 —— 横躺的 Lissajous */
                th = 6.2831853f * (float)el / (float)COM_EIGHT_PERIOD;
                cy = 9000L + (int32_t)((float)COM_EIGHT_AX * sinf(th));
                cp = 9000L + (int32_t)((float)COM_EIGHT_AY * sinf(2.0f * th));
            } else {
                /* 螺旋：半径 = R*sin^2(pi*u)，u=0 中心 / 0.5 最外 / 1 回中心 */
                float u = (float)el / (float)COM_SPIRAL_PERIOD;

                if (u > 1.0f) u = 1.0f;
                th = 6.2831853f * COM_SPIRAL_TURNS * u;
                r  = sinf(3.14159265f * u);
                r  = (float)COM_SPIRAL_RADIUS * r * r;
                cy = 9000L + (int32_t)(r * sinf(th));
                cp = 9000L + (int32_t)(r * cosf(th));
                if (u >= 1.0f) {
                    s_traj = TRAJ_OFF;
                    com_log("\r\nspiral end\r\n");
                }
            }

            s_target  = com_clamp(cy);
            s_target2 = com_clamp(cp);

            com_rpdo_pos_to(1, 0x000F, s_target);
            com_rpdo_pos_to(2, 0x000F, s_target2);
        }
    }

    /* ---- 连续正弦跟随：每 20ms 发一帧 RPDO ---- */
    if (s_sine_on) {
        uint32_t now = HAL_GetTick();

        if ((now - s_sine_last) >= COM_CSP_PERIOD) {
            uint32_t el = now - s_sine_t0;
            float    t;
            int32_t  pos;

            s_sine_last = now;

            if (el > COM_SINE_DURATION) {
                s_sine_on = 0;
                com_log("\r\nsine follow end, last target %ld.%02ld deg\r\n",
                        (long)(s_target / 100),
                        (long)(s_target < 0 ? (-s_target % 100) : (s_target % 100)));
                return;
            }

            /* 位置 = 中心 + 幅度 * sin(2*pi*t/T) */
            t   = (float)el / 1000.0f;
            pos = s_sine_c + (int32_t)((float)COM_SINE_AMP
                  * sinf(6.2831853f * t / ((float)COM_SINE_PERIOD / 1000.0f)));

            s_target = pos;
            com_rpdo_pos(0x000F, pos);

            /* 每 2 秒报一次，别刷屏（刷屏会拖慢 20ms 节拍） */
            if ((el % 2000u) < COM_CSP_PERIOD) {
                uint32_t v = 0;
                if (com_sdo_read(0x6064, 0, &v)) {
                    com_log("  %lu.%01lus  tgt %ld  act %ld.%02ld deg\r\n",
                            (unsigned long)(el / 1000), (unsigned long)((el % 1000) / 100),
                            (long)(pos / 100),
                            (long)((int32_t)v / 100),
                            (long)((int32_t)v < 0 ? (-(int32_t)v % 100) : ((int32_t)v % 100)));
                }
            }
        }
    }
}
