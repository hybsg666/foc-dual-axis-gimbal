"""单条位置命令的全过程监视：打印所有非遥测行 + 示波器四通道。

用法：python tools/step.py --target 10 --ms 1500
"""
import argparse
import time

import serial


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM3")
    ap.add_argument("--axis", type=int, default=0)
    ap.add_argument("--target", type=float, default=10)
    ap.add_argument("--ms", type=int, default=1500)
    a = ap.parse_args()

    s = serial.Serial()
    s.port = a.port
    s.baudrate = 115200
    s.timeout = 0.2
    s.dtr = False
    s.rts = False
    s.open()
    time.sleep(0.3)
    s.reset_input_buffer()

    s.write(("%dp%g\n" % (a.axis, 50)).encode())
    s.flush()
    time.sleep(2.8)
    s.reset_input_buffer()

    s.write(("0o%d\n" % a.ms).encode())
    s.flush()
    time.sleep(0.05)
    s.write(("%dp%g\n" % (a.axis, a.target)).encode())
    s.flush()

    P = V = C = I = None
    logs = []
    t0 = time.time()
    while time.time() - t0 < 12:
        line = s.readline().decode("utf8", "replace").strip()
        if not line:
            continue
        if line.startswith("[WAVE] P:"):
            P = [float(x) for x in line[9:].split()]
        elif line.startswith("[WAVE] V:"):
            V = [float(x) for x in line[9:].split()]
        elif line.startswith("[WAVE] C:"):
            C = [float(x) for x in line[9:].split()]
        elif line.startswith("[WAVE] I:"):
            I = [float(x) for x in line[9:].split()]
        elif not line[0].isdigit() and not line.startswith("[WAVE] "):
            logs.append(line)
        if P and V and C and I:
            break
    s.close()

    for l in logs[:25]:
        print("  " + l[:150])
    if P is None:
        print("no wave")
        return
    b = P[0]
    n = len(P)
    print("n=%d 起点 %.2f 终点 %.2f(相对起点 %+.2f) 目标 %g"
          % (n, b, P[-1], P[-1] - b, a.target))
    print("  t(ms)  位置(相对)   速度   指令    iq")
    st = max(1, n // 40)
    for i in range(0, n, st):
        print("%7d %10.2f %7.0f %7.0f %7.2f" % (i, P[i] - b, V[i], C[i], I[i]))


if __name__ == "__main__":
    main()
