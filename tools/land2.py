"""落地瞬态量化：单次跳跃 45->100，测"最后 1.5 度怎么走完"的指标。

指标（都用 1kHz 片上示波器原始点，时间按真实 loop 周期换算）：
  os   过冲（超过目标的度数，0 = 没冲）
  t05  从"速度掉到 60deg/s 以下"到"进入目标 ±0.5deg 并保持"的耗时
  pp   落地后 500ms 内位置峰峰（抖动量）
  vpp  同窗口速度峰峰
  osc  落地后速度的过零次数（振荡次数，"震几下"就是这个数）
"""
import argparse, statistics, time
import serial

R2D = 57.2958


def open_port(port):
    s = serial.Serial()
    s.port = port
    s.baudrate = 115200
    s.timeout = 0.2
    s.dtr = False
    s.rts = False
    s.open()
    time.sleep(0.3)
    s.reset_input_buffer()
    return s


def send(s, cmd, w=0.4):
    s.write((cmd + "\n").encode())
    s.flush()
    t = time.time()
    while time.time() - t < w:
        s.readline()


def grab(s, ms):
    # ★ 不要在这里发 'o'！trial() 已经在运动命令之前发过了；再发一次会把采集
    #   窗口重置到"跳跃中途"，P[0] 就不是起点 45°，base 和 err 全错（踩过）。
    P = V = C = None
    hdr = ""
    t0 = time.time()
    while time.time() - t0 < 10:
        l = s.readline().decode("utf8", "replace").strip()
        if l.startswith("[WAVE] P:"):
            P = [float(x) for x in l[9:].split()]
        elif l.startswith("[WAVE] V:"):
            V = [float(x) for x in l[9:].split()]
        elif l.startswith("[WAVE] C:"):
            C = [float(x) for x in l[9:].split()]
        elif l.startswith("[WAVE] m0"):
            hdr = l
        if P and V and C:
            return P, V, C, hdr
    return None, None, None, hdr


def trial(s, target, ms=1200):
    send(s, "0p45", 2.6)
    send(s, "0o%d" % ms, 0.05)
    send(s, "0b45@0,%d@3000;450" % target, 0.05)
    P, V, C, hdr = grab(s, ms)
    send(s, "0boff", 0.3)
    if P is None:
        return None
    base = P[0]
    tgt = float(target) - 45.0             # 相对目标
    rel = [x - base for x in P]
    # 起点必须是 45°（编码器会折到 -315° 这类等价角，所以按 mod 360 比）
    d0 = (base - 45.0 + 180.0) % 360.0 - 180.0
    if abs(d0) > 3.0:
        # 采集起点不在 45°，说明窗口没对上，这一轮数据不可信
        print("  [warn] base=%.1f° 偏离起点 45°，丢弃" % base)
        return None
    # 落地：最后一次"离目标还差 2° 以上"的点（比用速度阈值稳，不会被振铃时的
    #   瞬时大速度骗到；旧判据是"最后一次 |V|>60"，振铃本身就会超过 60 → 窗口选错）
    land = 0
    for i in range(len(rel) - 1, 0, -1):
        if abs(rel[i] - tgt) > 2.0:
            land = i
            break
    n = len(P)
    dt = ms / 1000.0 / n          # 真实每点时间（示波器按 loop 采样）
    overshoot = max(0.0, max(rel[land:]) - tgt)
    win = rel[land:]
    # t05：进入 target±0.5 后不再出来的时间
    t05 = None
    for i in range(len(win)):
        if all(abs(x - tgt) <= 0.5 for x in win[i:]):
            t05 = i * dt
            break
    w500 = min(len(win), int(0.5 / dt))
    pp = max(win[:w500]) - min(win[:w500])
    tv = V[land:land + w500]
    vpp = max(tv) - min(tv)
    zc = sum(1 for i in range(1, len(tv)) if (tv[i - 1] > 0) != (tv[i] > 0))
    return dict(os=overshoot, t05=t05 if t05 is not None else 99.0,
                pp=pp, vpp=vpp, osc=zc, dt=dt * 1000, err=rel[-1] - tgt)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM3")
    ap.add_argument("--target", type=float, default=100)
    ap.add_argument("--set", action="append", default=[], help="设置参数，可多次，如 k0.13,0.3")
    ap.add_argument("--rep", type=int, default=1)
    a = ap.parse_args()

    s = open_port(a.port)
    for c in a.set:
        send(s, "0" + c, 0.3)
    rows = []
    for _ in range(a.rep):
        r = trial(s, a.target)
        if r:
            rows.append(r)
    s.close()
    if not rows:
        print("no data")
        return
    f = lambda k: statistics.mean(x[k] for x in rows)
    print("set=%-22s os=%.2f°  t05=%.0fms  pp=%.2f°  vpp=%.0f°/s  osc=%d  err=%.2f°  (dt=%.1fms)"
          % ("|".join(a.set) or "-", f("os"), f("t05") * 1000, f("pp"), f("vpp"),
             round(f("osc")), f("err"), rows[0]["dt"]))
if __name__ == "__main__":
    main()
