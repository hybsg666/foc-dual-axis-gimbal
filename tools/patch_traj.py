# -*- coding: utf-8 -*-
"""给 F103 主站加 8 字 / 螺旋轨迹，并把 circle 合并成一个轨迹状态机。
文件是 GBK，注释保留中文，字符串全 ASCII。"""
import sys

p = 'STM32F103/Core/Src/canopen_master.c'
t = open(p, 'rb').read().decode('gbk')


def rep(old, new, must=1):
    global t
    n = t.count(old)
    if n < must:
        print('NOT FOUND (%d): %r' % (n, old[:70]))
        sys.exit(1)
    t = t.replace(old, new)


# ---------- 0. 之前已经打过这个补丁就不再重复 ----------
if 'TRAJ_EIGHT' in t:
    print('already patched')
    sys.exit(0)

# ---------- 1. 变量统一 ----------
rep("""static uint8_t  s_circ_on  = 0;      /* 画圆是否在跑 */
static uint32_t s_circ_t0  = 0;      /* 画圆起始时刻 */
static uint32_t s_circ_last= 0;      /* 画圆上一拍发送时刻 */""",
    """static uint8_t  s_traj     = 0;      /* 轨迹类型：0=停 1=圆 2=8 字 3=螺旋 */
static uint32_t s_traj_t0  = 0;      /* 轨迹起始时刻 */
static uint32_t s_traj_last= 0;      /* 轨迹上一拍发送时刻 */

/* 轨迹类型（s_traj 取值） */
#define TRAJ_OFF     0u
#define TRAJ_CIRCLE  1u
#define TRAJ_EIGHT   2u
#define TRAJ_SPIRAL  3u""")

# ---------- 2. 旧变量名全量替换（先长后短，避免互相吃掉） ----------
rep('s_circ_on   = 1;', 's_traj      = TRAJ_CIRCLE;')
rep('s_circ_on   = 0;', 's_traj      = 0;')
rep('s_circ_on = 0;',   's_traj = 0;')
rep('if (s_circ_on) {', 'if (s_traj) {')
rep('s_circ_t0',   's_traj_t0')
rep('s_circ_last', 's_traj_last')

# ---------- 3. 参数 ----------
rep("""#define COM_CIRC_RADIUS    3000L     /* 画圆半径 ±30°（单位 0.01°） */
#define COM_CIRC_PERIOD    8000u     /* 一圈 8 秒 */""",
    """#define COM_CIRC_RADIUS    3000L     /* 画圆半径 ±30°（单位 0.01°） */
#define COM_CIRC_PERIOD    8000u     /* 一圈 8 秒 */
#define COM_EIGHT_AX       3500L     /* 8 字 yaw 半幅 ±35° */
#define COM_EIGHT_AY       2500L     /* 8 字 pitch 半幅 ±25° */
#define COM_EIGHT_PERIOD   10000u    /* 一个完整 8 字 10 秒 */
#define COM_SPIRAL_RADIUS  3000L     /* 螺旋最大半径 ±30° */
#define COM_SPIRAL_TURNS   3.0f      /* 由内到外再收回，共转 3 圈 */
#define COM_SPIRAL_PERIOD  15000u    /* 整段螺旋 15 秒，走完自动停 */""")

# ---------- 4. circle 函数改成通用轨迹入口 ----------
rep("""/* 两轴画圆：以正前方 (yaw 90 / pitch 90) 为圆心，半径 COM_CIRC_RADIUS */
static void com_cmd_circle(void)
{
    if (s_traj) {
        s_traj = 0;
        com_log("circle stopped\\r\\n");
        return;
    }
    if (!s_dual) {
        com_log("  press b first (dual mode on)\\r\\n");
        return;
    }

    s_sine_on   = 0;
    s_traj_t0   = HAL_GetTick();
    s_traj_last = s_traj_t0;
    s_traj      = TRAJ_CIRCLE;

    com_log("circle start: r=30 deg, 8 s/rev, press c again to stop\\r\\n");
}""",
    """/* 轨迹：圆 / 8 字 / 螺旋，共用一台状态机。中心都在正前方 (90, 90)。
 * 同一个键再按一次 = 停；螺旋走满一圈后自己停。 */
static void com_cmd_traj(uint8_t mode)
{
    if (s_traj == mode) {                 /* 同一个键再按一次 -> 停 */
        s_traj = TRAJ_OFF;
        com_log("trajectory stopped\\r\\n");
        return;
    }
    if (!s_dual) {
        com_log("  press b first (dual mode on)\\r\\n");
        return;
    }

    s_sine_on   = 0;
    s_traj      = mode;
    s_traj_t0   = HAL_GetTick();
    s_traj_last = s_traj_t0;

    if (mode == TRAJ_CIRCLE) {
        com_log("circle start: r=30 deg, 8 s/rev, press c again to stop\\r\\n");
    } else if (mode == TRAJ_EIGHT) {
        com_log("figure-8 start: yaw +-35, pitch +-25 deg, 10 s/loop, press i again to stop\\r\\n");
    } else {
        com_log("spiral start: 3 turns 0 -> 30 -> 0 deg, 15 s, auto stop\\r\\n");
    }
}""")

# ---------- 5. 节拍：三种轨迹合一 ----------
rep("""    /* ---- 画圆：每 20ms 发一对 RPDO，两轴走圆 ---- */
    if (s_traj) {
        uint32_t now = HAL_GetTick();

        if ((now - s_traj_last) >= COM_CSP_PERIOD) {
            float   th;
            int32_t cy, cp;

            s_traj_last = now;
            th = 6.2831853f * (float)(now - s_traj_t0) / (float)COM_CIRC_PERIOD;

            cy = 9000L + (int32_t)((float)COM_CIRC_RADIUS * sinf(th));
            cp = 9000L + (int32_t)((float)COM_CIRC_RADIUS * cosf(th));

            s_target  = com_clamp(cy);
            s_target2 = com_clamp(cp);

            com_rpdo_pos_to(1, 0x000F, s_target);
            com_rpdo_pos_to(2, 0x000F, s_target2);
        }
    }""",
    """    /* ---- 轨迹（圆 / 8 字 / 螺旋）：每 20ms 发一对 RPDO ---- */
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
                    com_log("\\r\\nspiral end\\r\\n");
                }
            }

            s_target  = com_clamp(cy);
            s_target2 = com_clamp(cp);

            com_rpdo_pos_to(1, 0x000F, s_target);
            com_rpdo_pos_to(2, 0x000F, s_target2);
        }
    }""")

# ---------- 6. 按键 ----------
rep("    case 'c': case 'C': com_cmd_circle();      break;",
    """    case 'c': case 'C': com_cmd_traj(TRAJ_CIRCLE); break;
    case 'i': case 'I': com_cmd_traj(TRAJ_EIGHT);  break;
    case 'w': case 'W': com_cmd_traj(TRAJ_SPIRAL); break;""")

# ---------- 7. 帮助 ----------
rep('    com_log("  c   circle: both axes draw a circle (r=30 deg, 8 s/rev, press c to stop)\\r\\n");',
    """    com_log("  c   circle:   r=30 deg, 8 s/rev, press c to stop\\r\\n");
    com_log("  i   figure-8: yaw +-35, pitch +-25 deg, 10 s/loop, press i to stop\\r\\n");
    com_log("  w   spiral:   3 turns 0->30->0 deg, 15 s, auto stop\\r\\n");""")

open(p, 'wb').write(t.encode('gbk'))
print('patched ok, size %d' % len(t))
