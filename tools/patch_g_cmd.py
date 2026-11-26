# -*- coding: utf-8 -*-
"""新增指向命令 g<方位角> <仰角>
   实测：pitch 0 = 朝地、90 = 朝前水平、180 = 朝天  =>  pitch = 90 + 仰角
   约定：yaw 90 = 正前方，方位角 ±90 覆盖左右各 90 度
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

# 1) 记录"正在为哪条命令收参数"
rep('''static uint32_t s_line_ts  = 0;''',
'''static uint32_t s_line_ts  = 0;
static uint8_t  s_line_kind = 0;     /* 0 = a(绝对角度)   1 = g(指向) */''')

# 2) 插入 com_cmd_point（放在 com_cmd_abs 之后）
rep('''/* 提交缓冲区：回车和"静默超时"两条路都走这里 */''',
'''/* ============================================================
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
        com_log("  press b first (dual mode on)\\r\\n");
        return;
    }

    az = com_parse_float(&p, &ok_az);
    el = com_parse_float(&p, &ok_el);

    if (!ok_az) {
        com_log("  no number typed, nothing sent\\r\\n");
        return;
    }
    if (!ok_el) el = 0.0f;          /* 只给方位角：默认仰角 0（水平） */

    if (az < -90.0f) az = -90.0f;
    if (az >  90.0f) az =  90.0f;
    if (el < -90.0f) el = -90.0f;
    if (el >  90.0f) el =  90.0f;

    com_log("  POINT az %d  el %d deg\\r\\n", (int)az, (int)el);

    com_cmd_goto((int32_t)((90.0f + az * COM_POINT_YAW_SIGN) * 100.0f),
                 (int32_t)((90.0f + el) * 100.0f));
}

/* 提交缓冲区：回车和"= "两条路都走这里 */''')

# 3) submit 按命令类型分发
rep('''    s_line_on  = 0;
    s_line_len = 0;
    com_cmd_abs();              /* 读 s_line，内容仍在 */
}''',
'''    s_line_on  = 0;
    s_line_len = 0;

    if (s_line_kind == 1) com_cmd_point();
    else                  com_cmd_abs();     /* 读 s_line，内容仍在 */
}''')

# 4) a 命令设置 kind = 0
rep('''              s_line_on  = 1;
              s_line_len = 0;
              s_line_ts  = HAL_GetTick();
              com_log("  type yaw + space + pitch, e.g.  90 90=   (or Enter)\\r\\n");
              break;''',
'''              s_line_on   = 1;
              s_line_len  = 0;
              s_line_kind = 0;
              s_line_ts   = HAL_GetTick();
              com_log("  type yaw + space + pitch, e.g.  90 90=   (or Enter)\\r\\n");
              break;
    case 'g': case 'G':
              s_line_on   = 1;
              s_line_len  = 0;
              s_line_kind = 1;
              s_line_ts   = HAL_GetTick();
              com_log("  type azimuth + space + elevation, e.g.  30 15=   (0 0 = ahead level)\\r\\n");
              break;''')

# 5) help
rep('''    com_log("  a   absolute: a<yaw> <pitch>, end with '='  e.g. a45 90=  (= or Enter)\\r\\n");''',
'''    com_log("  a   absolute: a<yaw> <pitch>, end with '='  e.g. a45 90=  (= or Enter)\\r\\n");
    com_log("  g   point:    g<azimuth> <elevation>=   e.g. g30 15=   (0 0 = ahead, level)\\r\\n");''')

open(P, 'wb').write(t.encode('gbk'))
print('patched8 OK')
