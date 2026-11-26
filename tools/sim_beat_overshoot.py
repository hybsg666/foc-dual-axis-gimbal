# -*- coding: utf-8 -*-
"""
离线仿真：beat 跳跃的"到点过冲"从哪来，速度包络能不能治住。
全部用「度 / 秒」为单位，便于和遥测对照。

模型（刻意做得很简单，只保留起决定作用的两条）：
  1) 位置环：vel_cmd_pos = KP_POS * err_deg      （KP_POS=0.2 rad/s per deg → 11.46 /s）
  2) 速度环：实际速度一阶滞后跟随 vel_cmd，时间常数 TAU
     —— TAU 就是 VEL_FILT_TF(25ms) + 积分造成的等效延迟，这是过冲的元凶。
  3) 前馈 vff 按 quintic 导数给（可带末段渐撤）。
"""
import math

D2R = math.pi / 180.0

def quintic(u):
    u2 = u * u; u3 = u2 * u
    s  = u3 * (10 - 15 * u + 6 * u2)
    ds = 30 * u2 * (1 - u) ** 2
    return s, ds

def run(dp_deg=90.0, T=0.35, tau=0.025, kp=11.46, dt=1e-5,
        ramp_start=None, envelope_a=None, dwell_after=0.4):
    """跳一段 dp_deg，返回 (过冲度数, 到位时刻, 轨迹)"""
    dp = dp_deg
    n_move = int(T / dt)
    n_hold = int(dwell_after / dt)
    th = 0.0      # 实际位置（度）
    v  = 0.0      # 实际速度（度/秒）
    peak_over = 0.0
    t_settle = None
    traj = []
    for k in range(n_move + n_hold):
        if k < n_move:
            u = k / n_move
            s, ds = quintic(u)
            th_ref = dp * s
            vff = dp * ds / T
            if ramp_start is not None and u > ramp_start:      # 末段渐撤前馈
                r = (1.0 - u) / (1.0 - ramp_start)
                vff *= max(0.0, r)
            moving = True
        else:
            th_ref = dp
            vff = 0.0
            moving = False

        err = th_ref - th
        if abs(err) < 0.2:                # 位置环死区 POS_DEADBAND_DEG
            vpos = 0.0
        else:
            vpos = kp * err
        vcmd = vpos + vff

        if envelope_a is not None:        # ★ 速度包络：以当前速度必须能在剩余距离内停住
            d_rem = max(0.0, dp - th)
            v_safe = math.sqrt(2.0 * envelope_a * d_rem)
            if vcmd >  v_safe: vcmd =  v_safe
            if vcmd < -v_safe: vcmd = -v_safe

        v += (vcmd - v) * dt / tau        # 速度环一阶滞后
        th += v * dt

        over = th - dp
        if over > peak_over: peak_over = over
        if t_settle is None and not moving and abs(v) < 3.0 and abs(dp - th) < 1.0:
            t_settle = k * dt
        if k % 2000 == 0:
            traj.append((k * dt, th, v))
    return peak_over, t_settle, traj

print("=" * 68)
print("A) 复现现状：quintic + 末段25%撤前馈，无包络")
print("=" * 68)
for tau in (0.015, 0.025, 0.035):
    over, ts, _ = run(tau=tau, ramp_start=0.75)
    print(f"  速度环等效延迟 {tau*1000:4.0f}ms -> 过冲 {over:6.2f}°   到位 {ts if ts is None else round(ts,3)}s")

print()
print("=" * 68)
print("B) 理论校验：过冲 ≈ 峰值速度 × 延迟")
print("=" * 68)
vpk = 90 * 1.875 / 0.35
print(f"  quintic 峰值速度 = 1.875*Δp/T = {vpk:.0f} °/s")
for tau in (0.015, 0.025, 0.035):
    print(f"  {vpk:.0f} × {tau*1000:.0f}ms = {vpk*tau:.2f}°   ← 与 A 组结果对照")

print()
print("=" * 68)
print("C) 加速度包络 v_max = sqrt(2*a*d_rem)，扫描 a")
print("=" * 68)
for a in (3000, 5000, 8000, 12000, 20000):
    over, ts, _ = run(tau=0.025, ramp_start=None, envelope_a=a)
    print(f"  a={a:6d} °/s² -> 过冲 {over:6.2f}°   到位 {ts if ts is None else round(ts,3)}s")

print()
print("=" * 68)
print("D) 包络 + 末段撤前馈 组合")
print("=" * 68)
for a in (5000, 8000, 12000):
    for rs in (None, 0.75, 0.6):
        over, ts, _ = run(tau=0.025, ramp_start=rs, envelope_a=a)
        print(f"  a={a:6d}  ramp_start={str(rs):5s} -> 过冲 {over:6.2f}°  到位 {ts if ts is None else round(ts,3)}s")

print()
print("=" * 68)
print("E) 不同跳跃时间下的表现（选包络 a=8000）")
print("=" * 68)
for T in (0.30, 0.35, 0.45, 0.60):
    over0, ts0, _ = run(T=T, tau=0.025, ramp_start=0.75)
    over1, ts1, _ = run(T=T, tau=0.025, ramp_start=0.75, envelope_a=8000)
    print(f"  T={T:.2f}s  旧:过冲{over0:6.2f}° 到位{round(ts0,3) if ts0 else '-'}s"
          f"   | 加包络:过冲{over1:5.2f}° 到位{round(ts1,3) if ts1 else '-'}s")
