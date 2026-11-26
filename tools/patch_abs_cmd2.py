# -*- coding: utf-8 -*-
"""修正 a 命令的入参校验：空输入不发、只给一个数时另一个保持原目标。"""
import os, sys

P = os.path.abspath(os.path.join(os.path.dirname(__file__), '..',
                                 'STM32F103', 'Core', 'Src', 'canopen_master.c'))
t = open(P, 'rb').read().decode('gbk')

def rep(old, new):
    global t
    if t.count(old) != 1:
        print('FAIL anchor count =', t.count(old)); sys.exit(1)
    t = t.replace(old, new)

# --- 解析函数加 ok 输出 ---
rep('''static float com_parse_float(const char **pp)
{
    const char *p = *pp;
    float v = 0.0f, frac = 0.1f;
    int   neg = 0;

    while (*p == ' ' || *p == ',') p++;
    if (*p == '-') { neg = 1; p++; }
    else if (*p == '+') p++;

    while (*p >= '0' && *p <= '9') { v = v * 10.0f + (float)(*p - '0'); p++; }
    if (*p == '.') {
        p++;
        while (*p >= '0' && *p <= '9') {
            v += (float)(*p - '0') * frac;
            frac *= 0.1f;
            p++;
        }
    }

    *pp = p;
    return neg ? -v : v;
}''',
'''static float com_parse_float(const char **pp, int *ok)
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
}''')

# --- 调用处加校验 ---
rep('''    const char *p = s_line;
    float ay, ap;

    if (!s_dual) {
        com_log("  press b first (dual mode on), then a<yaw> <pitch>\\r\\n");
        return;
    }

    ay = com_parse_float(&p);
    ap = com_parse_float(&p);
''',
'''    const char *p = s_line;
    float ay, ap;
    int   ok_y = 0, ok_p = 0;

    if (!s_dual) {
        com_log("  press b first (dual mode on), then a<yaw> <pitch>\\r\\n");
        return;
    }

    ay = com_parse_float(&p, &ok_y);
    ap = com_parse_float(&p, &ok_p);

    if (!ok_y) {                    /* 空输入：什么都不发，别让两轴跑回 0 */
        com_log("  no number typed, nothing sent\\r\\n");
        return;
    }
    if (!ok_p) {                    /* 只给了 yaw：pitch 保持原目标不动 */
        ap = (float)s_target2 / 100.0f;
    }
''')

open(P, 'wb').write(t.encode('gbk'))
print('patched2 OK')
