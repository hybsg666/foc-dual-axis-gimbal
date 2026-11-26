# -*- coding: utf-8 -*-
"""新增笛卡尔指向： p<x> <y> <z>=
   坐标系（以云台为原点）：y = 正前方，x = 右方，z = 上方
   方位角 = atan2(x, y)，仰角 = atan2(z, sqrt(x^2+y^2))
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

# 1) 插入 com_cmd_xyz（放在 com_cmd_point 之后、com_line_submit 之前）
rep('''/* 提交缓冲区：回车和"= "两条路都走这里 */''',
'''/* ============================================================
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
        com_log("  press b first (dual mode on)\\r\\n");
        return;
    }

    x = com_parse_float(&p, &ok_x);
    y = com_parse_float(&p, &ok_y);
    z = com_parse_float(&p, &ok_z);

    if (!ok_x || !ok_y) {
        com_log("  need at least x and y, e.g.  100 200 50=\\r\\n");
        return;
    }
    if (!ok_z) z = 0.0f;

    r = sqrtf(x * x + y * y);
    if (r < 0.0001f && fabsf(z) < 0.0001f) {
        com_log("  that is the origin itself, ignored\\r\\n");
        return;
    }

    az = atan2f(x, y) * 57.2957795f;
    el = atan2f(z, r) * 57.2957795f;

    com_log("  XYZ %d,%d,%d -> az %d el %d\\r\\n",
            (int)x, (int)y, (int)z, (int)az, (int)el);

    if (az > 90.0f || az < -90.0f) {
        com_log("  note: azimuth beyond +-90, will hit yaw limit\\r\\n");
    }

    com_cmd_goto((int32_t)((90.0f + az) * 100.0f),
                 (int32_t)((90.0f + el) * 100.0f));
}

/* 提交缓冲区：回车和"= "两条路都走这里 */''')

# 2) submit 分发
rep('''    if (s_line_kind == 1) com_cmd_point();
    else                  com_cmd_abs();     /* 读 s_line，内容仍在 */''',
'''    if      (s_line_kind == 1) com_cmd_point();
    else if (s_line_kind == 2) com_cmd_xyz();
    else                       com_cmd_abs();   /* 读 s_line，内容仍在 */''')

# 3) 加 case 'p'
rep('''    case 'g': case 'G':''',
'''    case 'p': case 'P':
              s_line_on   = 1;
              s_line_len  = 0;
              s_line_kind = 2;
              s_line_ts   = HAL_GetTick();
              com_log("  type x + space + y + space + z, e.g.  100 200 50=   (y=ahead x=right z=up)\\r\\n");
              break;
    case 'g': case 'G':''')

# 4) help
rep('''    com_log("  g   point:    g<azimuth> <elevation>=   e.g. g30 15=   (0 0 = ahead, level)\\r\\n");''',
'''    com_log("  g   point:    g<azimuth> <elevation>=   e.g. g30 15=   (0 0 = ahead, level)\\r\\n");
    com_log("  p   aim at XYZ: p<x> <y> <z>=   e.g. p100 200 50=   (y ahead, x right, z up)\\r\\n");''')

open(P, 'wb').write(t.encode('gbk'))
print('patched9 OK')
