#!/usr/bin/env python3
"""beat_check.py - 按真实节奏跑定点跳跃，逐段量化「落地到位 + 残余抖动」

用法:
  python tools/beat_check.py --cmd "0b45@600,128@600;450"
  python tools/beat_check.py --cmd "..." --set "e1000"   # 先设参数

输出每段驻留的：驻留时长 / 到位误差 / 残余峰峰 / 速度峰峰
  · 到位误差 = 该段最末 50ms 的平均位置 − 目标（正=冲过头，负=还没走到）
  · 残余峰峰 = 该段最末 200ms 的位置峰峰（"看着还抖不抖"）
"""
import time, argparse
import serial

def open_port(port="COM3"):
    s = serial.Serial()
    s.port = port; s.baudrate = 115200; s.timeout = 0.1
    s.dtr = False; s.rts = False
    s.open(); time.sleep(0.15); s.reset_input_buffer()
    return s

def send(s, cmd, wait=0.3):
    s.write((cmd + "\n").encode()); s.flush()
    t = time.time()
    while time.time() - t < wait:
        s.readline()

def capture(s, cmd, ms):
    s.write(f"0o{ms}\n".encode()); s.flush(); time.sleep(0.05)
    s.write((cmd + "\n").encode()); s.flush()
    P = V = None
    t0 = time.time()
    while time.time() - t0 < ms / 1000.0 + 5.0:
        l = s.readline().decode("utf-8", "replace").strip()
        if l.startswith("[WAVE] P:"): P = [float(x) for x in l[9:].split()]
        elif l.startswith("[WAVE] V:"): V = [float(x) for x in l[9:].split()]
        if P is not None and V is not None: break
    return P, V

def analyze(P, V):
    n = min(len(P), len(V))
    P, V = P[:n], V[:n]
    # 用速度判定"在动"：|V| > 50°/s
    moving = [abs(v) > 50 for v in V]
    i = 0
    rows = []
    while i < n:
        if not moving[i]:
            j = i
            while j < n and not moving[j]:
                j += 1
            if (j - i) >= 10:                        # 驻留 ≥100ms（每点=10ms）才算一段
                seg_p = P[i:j]
                seg_v = V[i:j]
                tgt = round(sum(seg_p[-10:]) / min(10, len(seg_p[-10:])))
                err = sum(seg_p[-5:]) / min(5, len(seg_p[-5:])) - tgt
                tail = seg_p[-20:]
                rows.append({
                    "dwell": (j - i) * 0.01,
                    "tgt": tgt,
                    "err": err,
                    "pp": max(tail) - min(tail),
                    "vpp": max(seg_v[-20:]) - min(seg_v[-20:]),
                })
            i = j
        else:
            i += 1
    return rows

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cmd", default="0b45@600,128@600;450")
    ap.add_argument("--ms", type=int, default=2900)
    ap.add_argument("--set", default="")
    ap.add_argument("--start", default="0p45")
    a = ap.parse_args()

    s = open_port()
    if a.set:
        for one in a.set.split("|"):
            send(s, "0" + one.strip(), 0.35)
    send(s, a.start, 2.0)
    P, V = capture(s, a.cmd, a.ms)
    s.close()
    if not P:
        print("采集失败")
        return
    rows = analyze(P, V)
    print(f"{'目标':>7}{'驻留':>8}{'到位误差':>10}{'残余峰峰':>10}{'速度峰峰':>10}")
    print("-" * 46)
    for r in rows:
        print(f"{r['tgt']:>6}°{r['dwell']:>7.2f}s{r['err']:>+9.2f}°"
              f"{r['pp']:>9.2f}°{r['vpp']:>8.0f}°/s")

if __name__ == "__main__":
    main()
