# -*- coding: utf-8 -*-
"""按用户要求：
1) 去掉 800ms 自动提交（他明确说不要，必须按回车才发）
2) 新增三个单键命令，彻底绕开"输数字"这件事：
     0 -> 两轴都到 0 度
     9 -> 两轴都到 90 度
     8 -> 两轴都到 180 度
3) com_clamp 上限 170 -> 180（软限位本来就是 0~180）
"""
import os, sys

P = os.path.abspath(os.path.join(os.path.dirname(__file__), '..',
                                 'STM32F103', 'Core', 'Src', 'canopen_master.c'))
t = open(P, 'rb').read().decode('gbk')

def rep(old, new):
    global t
    if t.count(old) != 1:
        print('FAIL anchor count =', t.count(old))
        print(repr(old[:80]))
        sys.exit(1)
    t = t.replace(old, new)

# 1) 去掉自动提交
rep('''    /* ---- 收参数时 800ms 无新字符 -> 自动提交（不依赖串口助手会不会发回车）---- */
    if (s_line_on && s_line_len && (HAL_GetTick() - s_line_ts) > 800u) {
        com_log("\\r\\n");
        com_line_submit();
    }
''', '')

# 2) clamp 上限放宽到 180
rep('''    if (v100 >  17000L) return  17000L;
    if (v100 < -17000L) return -17000L;''',
'''    if (v100 >  18000L) return  18000L;
    if (v100 < -18000L) return -18000L;''')

# 3) 抽出 com_cmd_goto（放在 com_cmd_abs 前面）
rep('''static void com_cmd_abs(void)
{
    const char *p = s_line;''',
'''/* 两轴直接去指定角度（单位 0.01 度） */
static void com_cmd_goto(int32_t y100, int32_t p100)
{
    if (!s_dual) {
        com_log("  press b first (dual mode on)\\r\\n");
        return;
    }

    s_target  = com_clamp(y100);
    s_target2 = com_clamp(p100);

    com_rpdo_pos_to(1, 0x000F, s_target);
    com_rpdo_pos_to(2, 0x000F, s_target2);

    com_log("  ABS 0x201+0x202\\r\\n");
    com_log_deg("yaw  ", s_target);
    com_log_deg("pitch", s_target2);
}

static void com_cmd_abs(void)
{
    const char *p = s_line;''')

# 4) com_cmd_abs 尾部改成调 com_cmd_goto
rep('''    if (ay < -170.0f) ay = -170.0f;
    if (ay >  170.0f) ay =  170.0f;
    if (ap < -170.0f) ap = -170.0f;
    if (ap >  170.0f) ap =  170.0f;

    s_target  = (int32_t)(ay * 100.0f);
    s_target2 = (int32_t)(ap * 100.0f);

    com_rpdo_pos_to(1, 0x000F, s_target);
    com_rpdo_pos_to(2, 0x000F, s_target2);

    com_log("  ABS 0x201+0x202\\r\\n");
    com_log_deg("yaw  ", s_target);
    com_log_deg("pitch", s_target2);
}''',
'''    com_cmd_goto((int32_t)(ay * 100.0f), (int32_t)(ap * 100.0f));
}''')

# 5) 加三个单键
rep('''    case '1': s_node = 1; s_enabled = 0; s_sine_on = 0; s_dual = 0;''',
'''    case '0': com_cmd_goto(0, 0);             break;
    case '9': com_cmd_goto(9000, 9000);       break;
    case '8': com_cmd_goto(18000, 18000);     break;
    case '1': s_node = 1; s_enabled = 0; s_sine_on = 0; s_dual = 0;''')

# 6) help
rep('''    com_log("  a   absolute: a<yaw> <pitch> deg, Enter to send (dual mode)\\r\\n");''',
'''    com_log("  0   both axes to 0 deg      9   both to 90 deg\\r\\n");
    com_log("  8   both axes to 180 deg\\r\\n");
    com_log("  a   absolute: a<yaw> <pitch> deg, Enter to send (dual mode)\\r\\n");''')

open(P, 'wb').write(t.encode('gbk'))
print('patched4 OK')
