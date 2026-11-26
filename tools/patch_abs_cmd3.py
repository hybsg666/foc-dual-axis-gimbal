# -*- coding: utf-8 -*-
"""a 命令健壮化：
1) 不收回车也能发（800ms 没新字符自动提交）——很多串口助手默认不勾"发送新行"
2) 边输边回显，看得见自己敲了什么
3) 提交逻辑抽成 com_line_submit()，回车和超时共用
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

# 1) 增加时间戳变量
rep('''static char    s_line[24];
static uint8_t s_line_len = 0;
static uint8_t s_line_on  = 0;''',
'''static char     s_line[24];
static uint8_t  s_line_len = 0;
static uint8_t  s_line_on  = 0;
static uint32_t s_line_ts  = 0;      /* 最后一个字符到达的时刻，用于无回车自动提交 */''')

# 2) 在 com_cmd_abs 之后加 com_line_submit
rep('''static void com_cmd_sine(void)
{
    if (s_sine_on) {''',
'''/* 提交缓冲区：回车和"静默超时"两条路都走这里 */
static void com_line_submit(void)
{
    s_line[s_line_len] = 0;
    s_line_on  = 0;
    s_line_len = 0;
    com_cmd_abs();              /* 读 s_line，内容仍在 */
}

static void com_cmd_sine(void)
{
    if (s_sine_on) {''')

# 3) com_cmd 的收参分支：回显 + 时间戳 + 走 submit
rep('''    if (s_line_on) {
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
    }''',
'''    if (s_line_on) {
        if (c == '\\r' || c == '\\n') {     /* 有回车：立刻发 */
            com_line_submit();
            return;
        }
        if (c == 8 || c == 127) {           /* 退格 */
            if (s_line_len) s_line_len--;
            com_log("\\b \\b");
            s_line_ts = HAL_GetTick();
            return;
        }
        if (s_line_len < (uint8_t)(sizeof(s_line) - 1)) {
            s_line[s_line_len++] = (char)c;
            com_log("%c", c);               /* 回显，不然看不见自己敲了啥 */
        }
        s_line_ts = HAL_GetTick();
        return;
    }''')

# 4) 按下 a 时记时间戳
rep('''              s_line_on  = 1;
              s_line_len = 0;
              com_log("  type: yaw pitch (deg), e.g. 45 90 , then Enter\\r\\n");
              break;''',
'''              s_line_on  = 1;
              s_line_len = 0;
              s_line_ts  = HAL_GetTick();
              com_log("  type: yaw pitch (deg), e.g. 45 90  (Enter or just pause)\\r\\n");
              break;''')

# 5) com_poll 里加静默超时提交
rep('''void com_poll(void)
{
    uint8_t c;
''',
'''void com_poll(void)
{
    uint8_t c;

    /* ---- 收参数时 800ms 无新字符 -> 自动提交（不依赖串口助手会不会发回车）---- */
    if (s_line_on && s_line_len && (HAL_GetTick() - s_line_ts) > 800u) {
        com_log("\\r\\n");
        com_line_submit();
    }
''')

open(P, 'wb').write(t.encode('gbk'))
print('patched3 OK')
