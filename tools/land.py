#!/usr/bin/env python3
"""land.py - 采集定点跳跃遥测，量化"落地振荡"

用法:
  python tools/land.py --cmd "0b45@1500,128@1500;450" -d 8
输出：
  每段落地后的 峰峰值 / 首个峰值 / 衰减到 ±0.4° 所需时间 / 振荡频率
"""
import sys, time, math, argparse
import serial

D2R = math.pi / 180.0
R2D = 180.0 / math.pi

def run(port, cmd, dur, pre_wait):
    s = serial.Serial()
    s.port = port; s.baudrate = 115200; s.timeout = 0.05
    s.dtr = False; s.rts = False      # ★ 必须在 open() 前，否则会复位板子
    s.open()
    time.sleep(0.2); s.reset_input_buffer()
    if cmd:
        s.write((cmd + "\n").encode()); s.flush()
    rows = []
    t0 = time.time()
    while time.time() - t0 < dur:
        ln = s.readline().decode("utf-8", "replace").strip()
        if not ln: continue
        p = ln.split(",")
        if len(p) != 14: continue
        try:
            v = [float(x) for x in p]
        except ValueError:
            continue
        rows.append((time.time() - t0, v))
    s.close()
    return rows

def wrap(d):
    """把角度差折到 (-180, 180]"""
    return (d + 180.0) % 360.0 - 180.0

def analyze(rows, idx_mode=0, name="m0"):
    """idx_mode: 0 = 电机0, 7 = 电机1"""
    seg = [(t, v[idx_mode], v[idx_mode+1]*R2D, v[idx_mode+2]*R2D,
            v[idx_mode+3], v[idx_mode+4]) for t, v in rows]
    # 位置折成相对首点的等效角，避免编码器跨零点（如 45° 读成 -315°）
    if seg:
        base = seg[0][3]
        seg = [(s[0], s[1], s[2], base + wrap(s[3] - base), s[4], s[5]) for s in seg]
    # seg: (t, mode, target_deg, pos_deg, vel, iq)
    if not seg: return
    print(f"\n=== {name}  采样 {len(seg)} 点, 时长 {seg[-1][0]-seg[0][0]:.2f}s, "
          f"平均 {len(seg)/(seg[-1][0]-seg[0][0]):.0f}Hz ===")

    # 目标变化点 = 段边界
    tgt = [s[2] for s in seg]
    bounds = [0]
    for i in range(1, len(seg)):
        if abs(tgt[i] - tgt[i-1]) > 0.5:
            bounds.append(i)
    bounds.append(len(seg))

    for bi in range(len(bounds) - 1):
        a, b = bounds[bi], bounds[bi+1]
        if b - a < 5: continue
        T = tgt[a]
        # 找"首次进入 ±0.5°"的时刻
        hit = None
        for i in range(a, b):
            if abs(seg[i][3] - T) < 0.5:
                hit = i; break
        if hit is None:
            print(f"  段{bi+1}: target={T:.1f}° 未进入 ±0.5°")
            continue
        # hit 之后到段末
        tail = seg[hit:b]
        err = [s[3] - T for s in tail]
        # 峰峰 / 首个峰值
        pp = max(err) - min(err)
        # 衰减时间：从 hit 起，找最后一个 |err|>0.4 的点
        last_bad = 0
        for k, e in enumerate(err):
            if abs(e) > 0.4: last_bad = k
        settle = tail[last_bad][0] - tail[0][0]
        # 过零数 → 频率
        zc = sum(1 for k in range(1, len(err)) if err[k-1]*err[k] < 0)
        dur_tail = tail[-1][0] - tail[0][0]
        f = zc / (2*dur_tail) if dur_tail > 0.05 else 0
        maxe = max(abs(e) for e in err)
        print(f"  段{bi+1}: target={T:7.1f}°  进入±0.5°用时 {tail[0][0]-seg[a][0]:.2f}s  "
              f"落地峰峰 {pp:5.2f}°  最大偏差 {maxe:5.2f}°  "
              f"衰减<0.4° 用 {settle:.2f}s  振荡~{f:.1f}Hz")
        # 打印落地后 1.2s 的位置轨迹（每 0.1s 抽样）
        print("     轨迹:", end="")
        t_start = tail[0][0]
        k = 0
        for j in range(len(tail)):
            if tail[j][0] - t_start >= k * 0.1:
                print(f" {err[j]:+.1f}", end="")
                k += 1
                if k > 12: break
        print()

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--cmd", default="")
    ap.add_argument("-d", "--dur", type=float, default=8.0)
    ap.add_argument("-p", "--port", default="COM3")
    ap.add_argument("--axis", default="0", help="0 或 1 或 both")
    a = ap.parse_args()
    rows = run(a.port, a.cmd, a.dur, 0)
    if a.axis in ("0", "both"): analyze(rows, 0, "M0")
    if a.axis in ("1", "both"): analyze(rows, 7, "M1")
