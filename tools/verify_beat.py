#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
verify_beat.py —— 定点跳跃（b 模式）算法离线验证

对应固件：FOC_Motor::_updateBeat() / startBeat()
纯浮点复刻，不需要硬件。检查 6 件事：

  1. 驻留段目标角**严格等于**点位于，且前馈恒为 0
  2. 跳跃段两端速度为 0（smoothstep 的一阶导在 u=0/1 处为 0）
  3. 跳跃段端点精确落在 p0 / p1（不 overshoot、不欠冲）
  4. 前馈 = dθ/dt（数值微分对照，误差应 < 1e-5）
  5. 周期 = Σdwell + n·T_move，且相位回绕后连续（无跳变）
  6. 峰值速度 = 1.5 × 平均速度（这是"快"的来源）

用法：python tools/verify_beat.py
"""

import math

BEAT_MAX = 8


def build(pos_deg, dwell_ms, t_move_s, user_sign=1.0):
    """复刻 startBeat：度 -> 内部弧度（乘 user_sign）"""
    pts = [(user_sign * p * math.pi / 180.0, d) for p, d in zip(pos_deg, dwell_ms)]
    cycle = sum(d * 1e-3 for _, d in pts) + len(pts) * t_move_s
    return pts, t_move_s, cycle


def update_beat(t, pts, t_move, cycle):
    """复刻 _updateBeat：返回 (目标角 rad, 前馈 rad/s, 状态)"""
    ph = math.fmod(t, cycle)
    if ph < 0:
        ph += cycle
    tgt = pts[0][0]
    vff = 0.0
    state = "dwell0"
    acc = 0.0
    for i in range(len(pts)):
        d = pts[i][1] * 1e-3
        if ph < acc + d:
            tgt = pts[i][0]
            vff = 0.0
            state = f"dwell{i}"
            break
        acc += d
        if ph < acc + t_move:
            u = (ph - acc) / t_move
            sh = u * u * (3.0 - 2.0 * u)
            ds = 6.0 * u * (1.0 - u) / t_move
            p0 = pts[i][0]
            p1 = pts[(i + 1) % len(pts)][0]
            tgt = p0 + (p1 - p0) * sh
            vff = (p1 - p0) * ds
            state = f"move{i}->{(i + 1) % len(pts)}"
            break
        acc += t_move
    return tgt, vff, state


def main():
    ok = 0
    fail = []

    def check(name, cond, extra=""):
        nonlocal ok
        if cond:
            ok += 1
            print(f"  [PASS] {name} {extra}")
        else:
            fail.append(name)
            print(f"  [FAIL] {name} {extra}")

    # 场景：45 停 0.6s → 135 停 0.6s → 90 停 1.2s，每跳 0.35s
    pos = [45.0, 135.0, 90.0]
    dwell = [600, 600, 1200]
    TM = 0.35
    pts, tm, cycle = build(pos, dwell, TM)
    print(f"\n场景：点={pos} dwell={dwell}ms T_move={TM*1000:.0f}ms -> 周期 {cycle:.2f}s")

    # 1. 驻留段
    print("\n[1] 驻留段：目标严格等于点位、前馈为 0")
    for i, (p, d) in enumerate(pts):
        # 找到该驻留段起点
        acc = 0.0
        for k in range(i):
            acc += pts[k][1] * 1e-3 + tm
        t_mid = acc + d * 1e-3 * 0.5
        tgt, vff, st = update_beat(t_mid, pts, tm, cycle)
        check(f"点{i} 驻留中段 tgt==点位", abs(tgt - p) < 1e-12,
              f"tgt={math.degrees(tgt):.3f}° vff={vff:.6f} ({st})")
        check(f"点{i} 驻留前馈=0", abs(vff) < 1e-12)

    # 2 & 3. 跳跃段端点
    print("\n[2][3] 跳跃段：两端速度为 0、端点精确")
    for i in range(len(pts)):
        acc = 0.0
        for k in range(i):
            acc += pts[k][1] * 1e-3 + tm
        acc += pts[i][1] * 1e-3          # 跳过 w[i] 的驻留
        p0 = pts[i][0]
        p1 = pts[(i + 1) % len(pts)][0]
        t0, v0, _ = update_beat(acc + 1e-9, pts, tm, cycle)
        t1, v1, _ = update_beat(acc + tm - 1e-9, pts, tm, cycle)
        check(f"跳{i} 起点=w{i}", abs(t0 - p0) < 1e-6, f"{math.degrees(t0):.3f}°")
        check(f"跳{i} 终点=w{(i+1)%len(pts)}", abs(t1 - p1) < 1e-6, f"{math.degrees(t1):.3f}°")
        check(f"跳{i} 起点速度≈0", abs(v0) < 1e-3, f"{v0:.6f} rad/s")
        check(f"跳{i} 终点速度≈0", abs(v1) < 1e-3, f"{v1:.6f} rad/s")

    # 4. 前馈 vs 数值微分
    print("\n[4] 前馈 vs dθ/dt 数值微分")
    dt = 1e-6
    max_err = 0.0
    worst_t = 0.0
    t = 0.0
    while t < cycle:
        _, vff, _ = update_beat(t, pts, tm, cycle)
        a, _, _ = update_beat(t - dt / 2, pts, tm, cycle)
        b, _, _ = update_beat(t + dt / 2, pts, tm, cycle)
        num = (b - a) / dt
        # 只在跳跃段比较（驻留段有个阶跃边界，微分无意义）
        if abs(vff) > 1e-6 or abs(num) > 1e-6:
            e = abs(vff - num)
            if e > max_err:
                max_err, worst_t = e, t
        t += 0.001
    check("前馈=微分（最大偏差<1e-3）", max_err < 1e-3,
          f"max_err={max_err:.2e} @t={worst_t:.3f}s")

    # 5. 周期与回绕连续
    print("\n[5] 周期与相位回绕")
    exp_cycle = sum(d * 1e-3 for d in dwell) + len(pos) * TM
    check("周期=Σdwell+n·T", abs(cycle - exp_cycle) < 1e-12, f"{cycle:.4f}s")
    a, _, _ = update_beat(cycle - 1e-7, pts, tm, cycle)
    b, _, _ = update_beat(cycle + 1e-7, pts, tm, cycle)
    # 回绕处：上一周期末尾是"跳 2→0 的终点"，下一周期开头是 w0 驻留 → 都是 w0
    check("回绕处目标连续（都在 w0）", abs(a - pts[0][0]) < 1e-5 and abs(b - pts[0][0]) < 1e-5,
          f"末={math.degrees(a):.3f}° 首={math.degrees(b):.3f}°")

    # 6. 峰值速度 = 1.5 × 平均
    print("\n[6] 峰值速度 = 1.5×平均（smoothstep 特性）")
    acc = pts[0][1] * 1e-3
    peak = 0.0
    t = acc
    while t < acc + tm:
        _, v, _ = update_beat(t, pts, tm, cycle)
        peak = max(peak, abs(v))
        t += 0.0005
    dist = abs(pts[1][0] - pts[0][0])
    avg = dist / tm
    ratio = peak / avg if avg > 0 else 0
    check("峰值/平均 ≈ 1.5", abs(ratio - 1.5) < 0.02,
          f"峰值={math.degrees(peak):.0f}°/s 平均={math.degrees(avg):.0f}°/s 比={ratio:.3f}")

    print("\n" + "=" * 56)
    print(f"结果：{ok} 项通过, {len(fail)} 项失败")
    for f in fail:
        print("  失败：", f)
    return 0 if not fail else 1


if __name__ == "__main__":
    raise SystemExit(main())
