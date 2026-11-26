"""编码器在线监视：不驱动电机，只读角度。用手慢慢转轴，看数字跟不跟。

用法：
    python tools/enc_watch.py              # COM3，监视两轴
    python tools/enc_watch.py -a 1         # 只看 M1
    python tools/enc_watch.py -t 20        # 监视 20 秒

为什么用它（而不是看日志）：
  编码器 I2C 掉线时，固件会一直返回"上一次的角度"，所以 now 会冻结在某个值。
  肉眼盯着数字转轴，一动不变就是线断了 —— 比翻 Error 263 日志快 10 倍。

  它**只发 '?' 查询**，不发任何运动命令，电机不会通电动作，可以放心用手转。

判读：
  数字跟着手走           → 编码器正常
  数字完全不动           → I2C 没应答（供电/地/SDA/SCL 之一断了）
  数字偶尔整圈跳(±360)   → 磁铁松动或读数受干扰，需要看 [ENC] resync
  卡在 fault=0x01        → 过流已锁死，先断电查线，清故障用 '0r'/'1r'
"""
import argparse, re, sys, time

try:
    import serial
except ImportError:
    print("缺 pyserial： %s -m pip install pyserial" % sys.executable)
    sys.exit(1)

RE_NOW = re.compile(r"\[STAT\] m(\d).*?fault=0x([0-9A-Fa-f]{2}).*?"
                    r"target=([-0-9.]+)deg now=([-0-9.]+)deg")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-p", "--port", default="COM3")
    ap.add_argument("-a", "--axis", type=int, default=-1, help="-1 = 两轴都看")
    ap.add_argument("-t", "--sec", type=float, default=15.0)
    a = ap.parse_args()

    s = serial.Serial()
    s.port = a.port
    s.baudrate = 115200
    s.timeout = 0.15
    s.dtr = False       # ★ open() 之前
    s.rts = False
    s.open()
    time.sleep(0.3)
    s.reset_input_buffer()

    print("== 编码器监视 %s（只读，不驱动）==" % a.port)
    print("   现在用手【慢慢】转动轴，看下面数字是否跟着变\n")

    t0 = time.time()
    last_q = 0.0
    prev = {}
    moved = {0: 0.0, 1: 0.0}
    errs = 0
    while time.time() - t0 < a.sec:
        if time.time() - last_q > 0.35:
            s.write(b"?\n")
            s.flush()
            last_q = time.time()
        try:
            line = s.readline().decode("utf8", "replace").strip()
        except Exception:
            continue
        if "Error 263" in line or "bus error" in line:
            errs += 1
            continue
        m = RE_NOW.search(line)
        if not m:
            continue
        ax, flt, tgt, now = int(m.group(1)), int(m.group(2), 16), float(m.group(3)), float(m.group(4))
        if a.axis >= 0 and ax != a.axis:
            continue
        if ax in prev:
            d = now - prev[ax]
            if abs(d) < 90.0:       # 过滤整圈跳变
                moved[ax] += abs(d)
        prev[ax] = now
        flag = ""
        if flt:
            flag = "  FAULT 0x%02X%s" % (flt, " (过流)" if flt & 0x01 else "")
        print("  m%d  now=%8.2f°  target=%8.2f°  累计变化 %6.2f°%s"
              % (ax, now, tgt, moved[ax], flag))

    print("\n── 结果 ──")
    for ax in (0, 1):
        if ax not in prev:
            continue
        if a.axis >= 0 and ax != a.axis:
            continue
        if moved[ax] < 2.0:
            print("  m%d 累计变化仅 %.2f° → 【编码器没响应】或轴没转。"
                  % (ax, moved[ax]))
            print("     查：3.3V / GND / SDA / SCL 四根线，特别是插头压接处。")
        else:
            print("  m%d 累计变化 %.1f° → 编码器正常。" % (ax, moved[ax]))
    if errs:
        print("  期间 I2C 报错 %d 次 → 接触不稳，建议重压端子。" % errs)
    s.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
