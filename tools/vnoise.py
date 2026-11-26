"""测速噪声底：零扭矩 coast 下采集 1s，看位置峰峰与速度噪声。

用法：python tools/vnoise.py
"""
import statistics
import time

import serial


def main():
    s = serial.Serial()
    s.port = "COM3"
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

    send("0p50", 3.0)
    send("0o1000", 0.05)
    send("0i0", 0.05)
    P, V = scope(1000)
    send("0p50", 2.5)
    s.close()
    if P is None:
        print("no data")
        return
    print("coast 1s: 位置峰峰 %.4f deg  速度峰峰 %.0f deg/s  sigma %.1f"
          % (max(P) - min(P), max(V) - min(V), statistics.pstdev(V)))
    big = [(i, V[i]) for i in range(len(V)) if abs(V[i]) > 40]
    print("|V|>40 的点数:", len(big), " 前几个:", big[:12])
    print("位置前 30:", [f"{x:.3f}" for x in P[:30]])


if __name__ == "__main__":
    main()
