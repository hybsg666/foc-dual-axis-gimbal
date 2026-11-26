"""手动探针：发一条命令，每 200ms 打印一次状态，看它到底动不动。

用法：python tools/probe.py --cmd "0p30" --sec 3
"""
import argparse
import re
import time

import serial

PAT = (r"mode=(\w+) fault=(0x\w+) target=([-\d.]+)deg now=([-\d.]+)deg "
       r"err=([-+\d.]+)deg vel=([-\d.]+) iq=([-\d.]+)/([-\d.]+)(\s+SAT)?")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM3")
    ap.add_argument("--axis", type=int, default=0)
    ap.add_argument("--cmd", default="?")
    ap.add_argument("--sec", type=float, default=3.0)
    ap.add_argument("--step", type=float, default=0.25)
    a = ap.parse_args()

    s = serial.Serial()
    s.port = a.port
    s.baudrate = 115200
    s.timeout = 0.15
    s.dtr = False
    s.rts = False
    s.open()
    time.sleep(0.3)
    s.reset_input_buffer()

    def once(w=0.5):
        s.write(b"?\n")
        s.flush()
        t = time.time()
        while time.time() - t < w:
            line = s.readline().decode("utf8", "replace").strip()
            if "STAT" not in line or "^^" in line:
                continue
            if "m%d" % a.axis not in line:
                continue
            m = re.search(PAT, line)
            if m:
                return m
        return None

    s.write((a.cmd + "\n").encode())
    s.flush()
    t0 = time.time()
    while time.time() - t0 < a.sec:
        m = once()
        if m:
            print("%5.2fs mode=%-8s target=%7.1f now=%7.1f err=%+6.1f iq=%5.2f%s"
                  % (time.time() - t0, m.group(1), float(m.group(3)), float(m.group(4)),
                     float(m.group(5)), float(m.group(7)), " SAT" if m.group(9) else ""))
        time.sleep(a.step)
    s.close()


if __name__ == "__main__":
    main()
