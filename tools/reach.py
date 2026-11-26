"""行程探测：逐点下发位置，看实际到位与电流，判断哪里被卡住。

用法：python tools/reach.py --from 20 --to 100 --step 20
"""
import argparse
import re
import time

import serial


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM3")
    ap.add_argument("--axis", type=int, default=0)
    ap.add_argument("--from", dest="lo", type=float, default=20)
    ap.add_argument("--to", dest="hi", type=float, default=100)
    ap.add_argument("--step", type=float, default=20)
    ap.add_argument("--hold", type=float, default=2.5)
    a = ap.parse_args()

    s = serial.Serial()
    s.port = a.port
    s.baudrate = 115200
    s.timeout = 0.3
    s.dtr = False
    s.rts = False
    s.open()
    time.sleep(0.3)
    s.reset_input_buffer()

    def stat():
        s.write(b"?\n")
        s.flush()
        t = time.time()
        pat = r"target=([-\d.]+)deg now=([-\d.]+)deg err=([-+\d.]+)deg vel=([-\d.]+) iq=([-\d.]+)/([-\d.]+)(\s+SAT)?"
        while time.time() - t < 0.9:
            line = s.readline().decode("utf8", "replace").strip()
            if "STAT" in line and "m%d" % a.axis in line and "^^" not in line:
                m = re.search(pat, line)
                if m:
                    return m
        return None

    tgt = a.lo
    while tgt <= a.hi + 1e-6:
        s.write(("%dp%g\n" % (a.axis, tgt)).encode())
        s.flush()
        time.sleep(a.hold)
        m = stat()
        if not m:
            print("%6.1f  无状态" % tgt)
        else:
            print("目标 %6.1f -> now=%7.2f  err=%+6.2f  iq=%5.2f%s"
                  % (tgt, float(m.group(2)), float(m.group(3)), float(m.group(5)),
                     "  SAT!" if m.group(7) else ""))
        tgt += a.step
    s.close()


if __name__ == "__main__":
    main()
