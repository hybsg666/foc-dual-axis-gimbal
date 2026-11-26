"""定点跳跃的完整四通道轨迹（P/V/C/I），按 10ms 打印，用来看落地段细节。

用法：python tools/beatdump.py --cmd "0b45@0,100@3000;450" --ms 1500
"""
import argparse
import time

import serial


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM3")
    ap.add_argument("--axis", type=int, default=0)
    ap.add_argument("--cmd", default="0b45@0,100@3000;450")
    ap.add_argument("--ms", type=int, default=1500)
    ap.add_argument("--start", default="45")
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

    def send(c, w=0.4):
        s.write((c + "\n").encode())
        s.flush()
        t = time.time()
        while time.time() - t < w:
            s.readline()

    send("%dp%s" % (a.axis, a.start), 2.8)
    s.reset_input_buffer()
    s.write(("0o%d\n" % a.ms).encode())
    s.flush()
    time.sleep(0.05)
    s.write((a.cmd + "\n").encode())
    s.flush()

    P = V = C = I = None
    hdr = []
    t0 = time.time()
    while time.time() - t0 < 12:
        line = s.readline().decode("utf8", "replace").strip()
        if line.startswith("[WAVE] P:"):
            P = [float(x) for x in line[9:].split()]
        elif line.startswith("[WAVE] V:"):
            V = [float(x) for x in line[9:].split()]
        elif line.startswith("[WAVE] C:"):
            C = [float(x) for x in line[9:].split()]
        elif line.startswith("[WAVE] I:"):
            I = [float(x) for x in line[9:].split()]
        elif line.startswith("[WAVE] ") and not line.startswith("[WAVE] P"):
            hdr.append(line)
        if P and V and C and I:
            break
    send("0boff", 0.3)
    s.close()
    for l in hdr:
        print(l)
    if P is None:
        return
    b = P[0]
    n = len(P)
    print("n=%d 起点 %.2f" % (n, b))
    print("  t(ms)   位置(相对)    速度   指令    iq")
    for i in range(0, n, 10):
        print("%7d %12.2f %7.0f %6.0f %6.2f" % (i, P[i] - b, V[i], C[i], I[i]))


if __name__ == "__main__":
    main()
