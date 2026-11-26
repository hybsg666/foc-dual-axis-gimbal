# -*- coding: utf-8 -*-
"""新增 c 命令：两轴一起画圆（激光在墙上画个圈）
   yaw = 90 + R*sin(theta),  pitch = 90 + R*cos(theta)
"""
import os, sys

P = os.path.abspath(os.path.join(os.path.dirname(__file__), '..',
                                 'STM32F103', 'Core', 'Src', 'canopen_master.c'))
t = open(P, 'rb').read().decode('gbk')

def rep(old, new):
    global t
    if t.count(old) != 1:
        print('FAIL anchor count =', t.count(old)); sys.exit(1)
    t = t.replace(old, new)

# 1) 参数
rep('''#define COM_SINE_DURATION  20000u    /* 跑 20 秒自动停 */''',
'''#define COM_SINE_DURATION  20000u    /* 跑 20 秒自动停 */
#define COM_CIRC_RADIUS    3000L     /* 画圆半径 ±30°（单位 0.01°） */
#define COM_CIRC_PERIOD    8000u     /* 一圈 8 秒 */''')

# 2) 状态变量
rep('''static int32_t  s_sine_c   = 0;      /* 正弦中心（起跑时取当前位置） */''',
'''static int32_t  s_sine_c   = 0;      /* 正弦中心（起跑时取当前位置） */
static uint8_t  s_circ_on  = 0;      /* 画圆是否在跑 */
static uint32_t s_circ_t0  = 0;      /* 画圆起始时刻 */
static uint32_t s_circ_last= 0;      /* 画圆上一拍发送时刻 */''')

# 3) 命令函数（放在 com_cmd_sine 之前）
rep('''static void com_cmd_sine(void)
{
    if (s_sine_on) {''',
'''/* 两轴画圆：以正前方 (yaw 90 / pitch 90) 为圆心，半径 COM_CIRC_RADIUS */
static void com_cmd_circle(void)
{
    if (s_circ_on) {
        s_circ_on = 0;
        com_log("circle stopped\\r\\n");
        return;
    }
    if (!s_dual) {
        com_log("  press b first (dual mode on)\\r\\n");
        return;
    }

    s_sine_on   = 0;
    s_circ_t0   = HAL_GetTick();
    s_circ_last = s_circ_t0;
    s_circ_on   = 1;

    com_log("circle start: r=30 deg, 8 s/rev, press c again to stop\\r\\n");
}

static void com_cmd_sine(void)
{
    if (s_sine_on) {''')

# 4) case 'c'
rep('''    case 'b': case 'B': com_cmd_enable_both(); break;''',
'''    case 'c': case 'C': com_cmd_circle();      break;
    case 'b': case 'B': com_cmd_enable_both(); break;''')

# 5) 关掉 sine 时也关圆（互斥）
rep('''    s_sine_on   = 0;
    s_circ_t0   = HAL_GetTick();''',
'''    s_sine_on   = 0;
    s_circ_t0   = HAL_GetTick();''')

# 6) help
rep('''    com_log("  p   aim at XYZ: p<x> <y> <z>=   e.g. p100 200 50=   (y ahead, x right, z up)\\r\\n");''',
'''    com_log("  p   aim at XYZ: p<x> <y> <z>=   e.g. p100 200 50=   (y ahead, x right, z up)\\r\\n");
    com_log("  c   circle: both axes draw a circle (r=30 deg, 8 s/rev, press c to stop)\\r\\n");''')

# 7) com_poll 里跑圆
rep('''    /* ---- 连续正弦跟随：每 20ms 发一帧 RPDO ---- */
    if (s_sine_on) {''',
'''    /* ---- 画圆：每 20ms 发一对 RPDO，两轴走圆 ---- */
    if (s_circ_on) {
        uint32_t now = HAL_GetTick();

        if ((now - s_circ_last) >= COM_CSP_PERIOD) {
            float   th;
            int32_t cy, cp;

            s_circ_last = now;
            th = 6.2831853f * (float)(now - s_circ_t0) / (float)COM_CIRC_PERIOD;

            cy = 9000L + (int32_t)((float)COM_CIRC_RADIUS * sinf(th));
            cp = 9000L + (int32_t)((float)COM_CIRC_RADIUS * cosf(th));

            s_target  = com_clamp(cy);
            s_target2 = com_clamp(cp);

            com_rpdo_pos_to(1, 0x000F, s_target);
            com_rpdo_pos_to(2, 0x000F, s_target2);
        }
    }

    /* ---- 连续正弦跟随：每 20ms 发一帧 RPDO ---- */
    if (s_sine_on) {''')

# 8) disable / 切节点时停圆
rep('''static void com_cmd_disable(void)
{
    s_sine_on = 0;''',
'''static void com_cmd_disable(void)
{
    s_sine_on = 0;
    s_circ_on = 0;''')

rep('''    case '1': s_node = 1; s_enabled = 0; s_sine_on = 0; s_dual = 0;''',
'''    case '1': s_node = 1; s_enabled = 0; s_sine_on = 0; s_circ_on = 0; s_dual = 0;''')
rep('''    case '2': s_node = 2; s_enabled = 0; s_sine_on = 0; s_dual = 0;''',
'''    case '2': s_node = 2; s_enabled = 0; s_sine_on = 0; s_circ_on = 0; s_dual = 0;''')

# 9) 开正弦时停圆
rep('''    s_sine_c    = s_pos_now;        /* 以当前位置为中心，避免起跑台阶 */''',
'''    s_circ_on   = 0;
    s_sine_c    = s_pos_now;        /* 以当前位置为中心，避免起跑台阶 */''')

open(P, 'wb').write(t.encode('gbk'))
print('patched10 OK')
