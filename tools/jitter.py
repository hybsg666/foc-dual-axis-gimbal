"""静止抖动诊断：电机不动的时候到底在抖什么、抖多快、是位置在动还是只有电流在动。

用法：
    python tools/jitter.py                # 诊断 M1（默认）
    python tools/jitter.py -a 0           # 诊断 M0
    python tools/jitter.py -a 1 -p COM3

它做的事：
  1. 停掉该轴的 beat（'boff'），等 1s 静止
  2. 打印 [STAT] 一行（target/now/err/vel/iq/限位/是否 CLAMPED）
  3. 用 1kHz 片上示波器采 2s **静止**波形
  4. 分别给出 位置/速度/速度指令/iq 的峰峰值、标准差、主频
  5. 再关掉到位成形器（s0）重采一次 → 用来证明"这版改动跟它无关"

判读表（脚本会自动打印结论）：
  位置峰峰 大 + 速度有周期性       → 极限环（控制环自激）
  位置峰峰 小 + iq 峰峰 大         → 力矩纹波（结构没动，但你在听/摸到震动）
  iq 贴住限幅(SAT) + 位置不动      → 力矩不够或换相角错
  err 单向偏 + CLAMPED             → 目标被软限位钳住，环在顶边界
"""
import argparse, math, statistics, sys, time

try:
    import serial
except ImportError:
    print("缺 pyserial： pip install pyserial")
    sys.exit(1)


def open_port(p):
    s = serial.Serial()
    s.port = p
    s.baudrate = 115200
    s.timeout = 0.2
    s.dtr = False      # ★ 必须在 open() 之前
    s.rts = False
    s.open()
    time.sleep(0.3)
    s.reset_input_buffer()
    return s


def talk(s, cmd, w=0.4, collect=False):
    s.write((cmd + "\n").encode())
    s.flush()
    out = []
    t = time.time()
    while time.time() - t < w:
        l = s.readline().decode("utf8", "replace").strip()
        if l and collect:
            out.append(l)
    return out


def grab(s, ms):
    P = V = C = I = None
    t0 = time.time()
    while time.time() - t0 < 10:
        l = s.readline().decode("utf8", "replace").strip()
        if l.startswith("[WAVE] P:"):
            P = [float(x) for x in l[9:].split()]
        elif l.startswith("[WAVE] V:"):
            V = [float(x) for x in l[9:].split()]
        elif l.startswith("[WAVE] C:"):
            C = [float(x) for x in l[9:].split()]
        elif l.startswith("[WAVE] I:"):
            I = [float(x) for x in l[9:].split()]
        if P and V and I:
            return P, V, C, I
    return None, None, None, None


def stats(x):
    m = statistics.mean(x)
    sd = statistics.pstdev(x)
    return m, sd, max(x) - min(x)


def dom_freq(x, dt, fmin=1.0, fmax=60.0, step=0.5):
    """朴素 DFT 找主频：抖动是准正弦，找幅度最大的频点就够。"""
    n = len(x)
    if n < 32:
        return 0.0, 0.0
    m = statistics.mean(x)
    y = [v - m for v in x]
    best_f, best_a = 0.0, 0.0
    f = fmin
    while f <= fmax:
        w = 2.0 * math.pi * f
        re = 0.0
        im = 0.0
        for i, v in enumerate(y):
            a = w * i * dt
            re += v * math.cos(a)
            im += v * math.sin(a)
        amp = math.hypot(re, im) * 2.0 / n
        if amp > best_a:
            best_a, best_f = amp, f
        f += step
    return best_f, best_a


def measure(s, axis, sec):
    talk(s, "%dboff" % axis, 0.3)     # 确保没有 beat 在跑
    time.sleep(0.8)
    # 只启动示波器、不发任何运动命令 → 采到的就是"静止"状态
    s.write(("%do%d\n" % (axis, sec)).encode())
    s.flush()
    time.sleep(0.03)
    P, V, C, I = grab(s, sec)
    if P is None:
        return None
    return dict(P=P, V=V, C=C, I=I, dt=sec / 1000.0 / len(P))


def report(tag, d):
    P, V, C, I = d["P"], d["V"], d["C"], d["I"]
    dt = d["dt"]
    pm, ps, pp = stats(P)
    vm, vs, vp = stats(V)
    cm, cs, cp = stats(C)
    im, isd, ip = stats(I)
    f_pos, a_pos = dom_freq(P, dt)
    f_vel, a_vel = dom_freq(V, dt)
    f_iq, a_iq = dom_freq(I, dt)
    print("  [%s] n=%d  dt=%.2fms" % (tag, len(P), dt * 1000))
    print("       位置  均值 %8.2f°   σ %6.3f°   峰峰 %6.3f°" % (pm, ps, pp))
    print("       速度  均值 %8.1f°/s σ %6.1f°/s 峰峰 %6.0f°/s" % (vm, vs, vp))
    print("       速度指令   σ %6.1f°/s 峰峰 %6.0f°/s" % (cs, cp))
    print("       iq    均值 %8.3fA  σ %6.3fA  峰峰 %6.3fA" % (im, isd, ip))
    print("       主频  位置 %5.1fHz(幅度%5.3f°)  速度 %5.1fHz(%5.1f°/s)  iq %5.1fHz(%5.3fA)"
          % (f_pos, a_pos, f_vel, a_vel, f_iq, a_iq))
    return dict(ps=ps, pp=pp, vp=vp, ip=ip, isd=isd, im=im, f=f_vel, fp=f_pos, fi=f_iq)


def verdict(a, b, axis):
    print("")
    print("  ── 结论 ──")
    if a["pp"] < 0.08 and a["vp"] < 12:
        print("  位置几乎不动（峰峰 %.2f°），而 iq 峰峰 %.3fA / 主频 %.1fHz。"
              % (a["pp"], a["ip"], a["fi"]))
        print("  ⇒ 抖的是**力矩/电流**，不是位置。属于电磁纹波，听觉/触觉明显但机械没在动。")
        print("     方向：降速度环 P、加大速度滤波，或抬高电流环截止频率。")
    elif a["vp"] > 25 and a["pp"] > 0.3:
        print("  位置峰峰 %.2f°、速度峰峰 %.0f°/s、主频 %.1fHz ⇒ **极限环**（控制环自激）。"
              % (a["pp"], a["vp"], a["f"]))
        print("     方向：该轴速度环 P 与位置环 P 的组合在这个惯量下过冲 → 降 Pv 或降 Kp。")
    elif abs(a["im"]) > 0.4:
        print("  iq 均值 %.3fA 偏大 ⇒ 一直在顶力矩（重力/静摩擦）。" % a["im"])
        if a["pp"] < 0.3:
            print("     位置还算稳，说明是被卡在【顶住】的平衡点上小步调整。")
    else:
        print("  数据不典型，看上面的峰峰/主频组合自行判断。")
    print("")
    if b is None:
        return
    d_pp = b["pp"] - a["pp"]
    print("  成形器 ON vs OFF 对比：位置峰峰 %.2f° vs %.2f°（差 %+.2f°），"
          "iq 峰峰 %.3f vs %.3fA"
          % (a["pp"], b["pp"], d_pp, a["ip"], b["ip"]))
    if abs(d_pp) < 0.15 and abs(a["ip"] - b["ip"]) < 0.02:
        print("  ⇒ 两组几乎一样：**与这版到位成形器无关**，是既有问题。")
    else:
        print("  ⇒ 两组有差异：成形器对该轴有影响，需要单独调。")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-a", "--axis", type=int, default=1, help="0=M0, 1=M1（默认）")
    ap.add_argument("-p", "--port", default="COM3")
    ap.add_argument("-s", "--sec", type=int, default=2000, help="采集时长 ms，最大 3000")
    a = ap.parse_args()

    s = open_port(a.port)
    print("== 静止抖动诊断  M%d  %s ==" % (a.axis, a.port))

    # 状态行：能直接看出 目标/实际/误差/限位/是否被钳
    for l in talk(s, "%d?" % a.axis, 0.5, collect=True):
        if l.startswith("[STAT]"):
            print(" ", l)

    print("\n[1/2] 成形器 ON（当前固件默认）")
    d1 = measure(s, a.axis, a.sec)
    if d1 is None:
        print("  [X] 没收到 [WAVE]，检查串口/板子")
        s.close()
        return 1
    r1 = report("成形器ON", d1)

    print("\n[2/2] 成形器 OFF（'%de8000,s0'）" % a.axis)
    talk(s, "%de8000,s0" % a.axis, 0.4)
    d2 = measure(s, a.axis, a.sec)
    r2 = report("成形器OFF", d2) if d2 else None

    verdict(r1, r2, a.axis)
    talk(s, "%de8000,s344" % a.axis, 0.3)   # 恢复默认
    s.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
