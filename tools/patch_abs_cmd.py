# -*- coding: utf-8 -*-
"""给 F103 主站加一条带参数的命令：a<yaw> <pitch>（绝对定位，一次到位）。
文件是 GBK，全程用 gbk 读写，保持编码不变。
"""
import io, sys, os

P = os.path.join(os.path.dirname(__file__), '..', 'STM32F103', 'Core', 'Src', 'canopen_master.c')
P = os.path.abspath(P)
t = open(P, 'rb').read().decode('gbk')

def rep(old, new, cnt=1):
    global t
    n = t.count(old)
    if n != cnt:
        print('FAIL: anchor found %d times (expect %d):' % (n, cnt))
        print(repr(old[:120]))
        sys.exit(1)
    t = t.replace(old, new)

# ---------- 1) help 加一行 ----------
rep(
'    com_log("  1   node 1 (M0 yaw)     2   node 2 (M1 pitch)\\r\\n");',
'    com_log("  a   absolute: a<yaw> <pitch> deg, Enter to send (dual mode)\\r\\n");\n'
'    com_log("  1   node 1 (M0 yaw)     2   node 2 (M1 pitch)\\r\\n");'
)

# ---------- 2) 插入带参数命令的实现（放在 com_cmd_sine 之前）----------
BLOCK = r'''/* ============================================================
 * a<yaw> <pitch>  绝对定位（一次到位，不用按十几下 +/-）
 *   串口工具只能发单字符，所以这里自己攒一行：按 a 之后收到的字符
 *   都进缓冲区，回车才解析。其它单字符命令完全不受影响。
 * ============================================================ */
static char    s_line[24];
static uint8_t s_line_len = 0;
static uint8_t s_line_on  = 0;

/* 读一个十进制数（可带负号、小数点），*pp 会前移到数字之后 */
static float com_parse_float(const char **pp)
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
}

static void com_cmd_abs(void)
{
    const char *p = s_line;
    float ay, ap;

    if (!s_dual) {
        com_log("  press b first (dual mode on), then a<yaw> <pitch>\r\n");
        return;
    }

    ay = com_parse_float(&p);
    ap = com_parse_float(&p);

    if (ay < -170.0f) ay = -170.0f;
    if (ay >  170.0f) ay =  170.0f;
    if (ap < -170.0f) ap = -170.0f;
    if (ap >  170.0f) ap =  170.0f;

    s_target  = (int32_t)(ay * 100.0f);
    s_target2 = (int32_t)(ap * 100.0f);

    com_rpdo_pos_to(1, 0x000F, s_target);
    com_rpdo_pos_to(2, 0x000F, s_target2);

    com_log("  ABS 0x201+0x202\r\n");
    com_log_deg("yaw  ", s_target);
    com_log_deg("pitch", s_target2);
}

static void com_cmd_sine(void)'''

rep('static void com_cmd_sine(void)', BLOCK)

# ---------- 3) com_cmd 开头拦截参数输入 ----------
rep(
'''static void com_cmd(uint8_t c)
{
    switch (c) {''',
'''static void com_cmd(uint8_t c)
{
    /* a 之后进入"收参数"状态：攒一行，回车提交，退格可改 */
    if (s_line_on) {
        if (c == '\\r' || c == '\\n') {
            s_line[s_line_len] = 0;
            s_line_on  = 0;
            s_line_len = 0;
            com_cmd_abs();
            return;
        }
        if (c == 8 || c == 127) {
            if (s_line_len) s_line_len--;
            return;
        }
        if (s_line_len < (uint8_t)(sizeof(s_line) - 1)) s_line[s_line_len++] = (char)c;
        return;
    }

    switch (c) {'''
)

# ---------- 4) 加 case 'a' ----------
rep(
"    case 'b': case 'B': com_cmd_enable_both(); break;",
"    case 'b': case 'B': com_cmd_enable_both(); break;\n"
"    case 'a': case 'A':\n"
"              s_line_on  = 1;\n"
"              s_line_len = 0;\n"
"              com_log(\"  type: yaw pitch (deg), e.g. 45 90 , then Enter\\r\\n\");\n"
"              break;"
)

open(P, 'wb').write(t.encode('gbk'))
print('patched OK, size', len(t.encode('gbk')))
