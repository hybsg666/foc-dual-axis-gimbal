# -*- coding: utf-8 -*-
"""让 a 命令不依赖回车：加 '=' 作为确认键。
   a90 90=  ->  执行
   a90 90<Enter> -> 也执行（若串口助手会发回车）
   其余行为不变；= 只在"收参数"状态下拦截，不影响其它命令。
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

rep('''        if (c == '\\r' || c == '\\n') {     /* 有回车：立刻发 */
            com_line_submit();
            return;
        }''',
'''        if (c == '\\r' || c == '\\n' || c == '=') {   /* 回车或 = ：立刻发 */
            com_line_submit();
            return;
        }''')

rep('''              com_log("  type: yaw pitch (deg), e.g. 45 90  (Enter or just pause)\\r\\n");''',
'''              com_log("  type yaw + space + pitch, e.g.  90 90=   (or Enter)\\r\\n");''')

rep('''    com_log("  a   absolute: a<yaw> <pitch> deg, Enter to send (dual mode)\\r\\n");''',
'''    com_log("  a   absolute: a<yaw> <pitch>, end with '='  e.g. a45 90=  (= or Enter)\\r\\n");''')

open(P, 'wb').write(t.encode('gbk'))
print('patched5 OK')
