"""开环方向自检：给恒定 iq，看位置往哪边走。

用法：python tools/dirtest.py --iq 0.15 --ms 600
"""
import argparse
import time

import serial


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM3")
    ap.add_argument("--axis", type=int, default=0)
    ap.add_argument("--iq", type=float, default=0.15)
    ap.add_argument("--ms", type=int, default=600)
    ap.add_argument("--pre", type=float, default=50)
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

    def send(c, w=0.4):
        s.write((c + "\n").encode())
        s.flush()
        t = time.time()
        while time.time() - t < w:
            s.readline()

    def scope(ms):
        P = V = None
        t0 = time.time()
        while time.time() - t0 < 10:
            line = s.readline().decode("utf8", "replace").strip()
            if line.startswith("[WAVE] P:"):
                P = [float(x) for x in line[9:].split()]
            elif line.startswith("[WAVE] V:"):
                V = [float(x) for x in line[9:].split()]
            if P and V:
                return P, V
        return None, None

    send("%dp%g" % (a.axis, a.pre), 2.8)
    send("0o%d" % a.ms, 0.05)
    send("%di%g" % (a.axis, a.iq), 0.05)
    P, V = scope(a.ms)
    send("%di0" % a.axis, 0.05)
    send("%dp%g" % (a.axis, a.pre), 2.5)
    s.close()
    if P is None:
        print("no data")
        return
    print("iq=%+.2fA  %dms：起点 %.2f  终点 %.2f  位移 %+.2f deg  最大速度 %.0f deg/s"
          % (a.iq, a.ms, P[0], P[-1], P[-1] - P[0], max(abs(v) for v in V)))


if __name__ == "__main__":
    main()
