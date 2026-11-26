"""抓一条完整跳跃的所有非遥测日志，用于查偶发异常（ENC resync / vel spike / I2C）。

用法：python tools/trace.py --cmd "0b45@0,100@3000;450" --ms 1500
"""
import argparse
import time

import serial


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM3")
    ap.add_argument("--cmd", default="0b45@0,100@3000;450")
    ap.add_argument("--ms", type=int, default=1500)
    ap.add_argument("--rep", type=int, default=2)
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

    for r in range(a.rep):
        send("0p45", 2.8)
        s.reset_input_buffer()
        s.write(("0o%d\n" % a.ms).encode())
        s.flush()
        time.sleep(0.05)
        s.write((a.cmd + "\n").encode())
        s.flush()
        P = V = None
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
            elif line.startswith("[WAVE] "):
                if "落地" in line or "n=" in line:
                    logs.append(line)
            elif not line[0].isdigit():
                logs.append(line)
            if P and V:
                break
        send("0boff", 0.3)
        print("=== 第 %d 次 ===" % (r + 1))
        for l in logs:
            print("   " + l[:140])
        if P:
            print("   起点 %.2f 终点 %.2f  峰峰(全程) %.2f" % (P[0], P[-1], max(P) - min(P)))
    s.close()


if __name__ == "__main__":
    main()
