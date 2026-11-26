#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sim_damping.py —— 复刻固件三环，量化"为什么位置环 Kd 从 0.3 加到 1.0 没反应"

对应固件（4MytowFOC SVPWM YT）：
  位置环: vel_cmd = kp * err_deg,  err_deg < POS_DEADBAND_DEG(0.2) 时输出 0
          限幅 VEL_LIMIT = 30 rad/s
  Kd  : vel_cmd -= kd * velocity            (foc_motor.cpp 第 543-551 行)
  速度环: iq = Pv*e + Iv*∫e,  e = vel_cmd - velocity
          限幅 VEL_LIMIT_NEAR = 0.5 A
  电流环: 当理想一阶（tau ≈ 1ms）
  动力学: J*dω/dt = Kt*iq - b*ω

关键数学（这就是 Kd 无效的原因）：
  iq = Pv*(kp*err - kd*ω - ω) = Pv*kp*err - Pv*(kd+1)*ω
  → Kd 只能把"等效阻尼系数"从 Pv 抬到 Pv*(kd+1)
  → Pv = 0.02，kd=1.0 也只到 0.04，绝对增量 0.02
  → 而速度环刚度由 Iv=1.0 决定，阻尼比 ζ = Kt*Pv*(kd+1) / (2*J*ωn)
     几乎不变 ⇒ 看上去"完全没区别"
"""
import math
import numpy as np

# ---------------- 固件常量 ----------------
VEL_LIMIT      = 30.0      # rad/s  位置环输出限幅
VEL_LIMIT_NEAR = 0.5       # A      速度环输出限幅
POS_DEADBAND_DEG = 0.2
ENC_LSB = 2 * math.pi / 4096.0      # AS5600 12bit

# ---------------- 被控对象（估计值，做敏感性扫描） ----------------
KT = 0.10          # N·m/A   2208 云台电机力矩常数（量级估计）
B_FRIC = 0.0005    # N·m·s  粘性摩擦（轴承很滑，用户"手扶即停"）

DT = 0.0037        # s  实测 loop 周期


def simulate(j, kp_deg=0.2, kd=0.3, pv=0.02, iv=1.0,
             theta0_deg=6.0, t_end=6.0, noise_lsb=0.0, seed=1, tf_vel=0.01):
    """跑一次三环仿真，返回 (t, theta_deg, vel, iq)"""
    rng = np.random.default_rng(seed)
    n = int(t_end / DT)

    theta = math.radians(theta0_deg)     # 真值
    omega = 0.0
    integ = 0.0
    e_prev = 0.0
    iq_act = 0.0
    vel_f = 0.0                          # 固件 _velLowPass：y = a*y + (1-a)*x, a = Tf/(Tf+dt)
    alpha = tf_vel / (tf_vel + DT)

    th_q_prev = theta                    # 上一拍编码器读数（量化后）
    out = np.zeros((n, 4))

    for k in range(n):
        # --- 编码器（量化 + 可选抖动噪声）---
        th_q = math.floor(theta / ENC_LSB) * ENC_LSB
        if noise_lsb > 0:
            th_q += rng.normal(0, ENC_LSB * noise_lsb)
        vel_raw = (th_q - th_q_prev) / DT
        th_q_prev = th_q
        vel_f = alpha * vel_f + (1 - alpha) * vel_raw

        # --- 位置环（纯 P + 死区）---
        err_deg = (0.0 - th_q) * 180.0 / math.pi     # target = 0
        if abs(err_deg) < POS_DEADBAND_DEG:
            vel_cmd = 0.0
        else:
            vel_cmd = kp_deg * err_deg
        vel_cmd = max(-VEL_LIMIT, min(VEL_LIMIT, vel_cmd))

        # --- Kd（固件当前实现：从速度指令里减）---
        if kd > 0:
            vel_cmd -= kd * vel_f
            vel_cmd = max(-VEL_LIMIT, min(VEL_LIMIT, vel_cmd))

        # --- 速度环 PI（Tustin）---
        e = vel_cmd - vel_f
        integ += iv * DT * 0.5 * (e + e_prev)
        integ = max(-VEL_LIMIT_NEAR, min(VEL_LIMIT_NEAR, integ))
        e_prev = e
        iq = pv * e + integ
        iq = max(-VEL_LIMIT_NEAR, min(VEL_LIMIT_NEAR, iq))

        # --- 电流环一阶滞后 ---
        iq_act += (iq - iq_act) * (DT / (0.001 + DT))

        # --- 动力学 ---
        alpha_acc = (KT * iq_act - B_FRIC * omega) / j
        omega += alpha_acc * DT
        theta += omega * DT

        out[k] = (k * DT, theta * 180 / math.pi, omega, iq_act)

    return out


def report(name, arr):
    t, th, om, iq = arr.T
    peak = np.max(np.abs(th))
    # 最后一次超过 0.5°/1.0° 的时刻 = 衰减时间
    def settle(thr):
        idx = np.where(np.abs(th) > thr)[0]
        return t[idx[-1]] if len(idx) else 0.0
    # 过零次数估计振荡频率（前 2 秒）
    m = t < 2.0
    zc = np.sum(np.diff(np.sign(th[m])) != 0)
    f = zc / 2.0 / max(t[m][-1], 1e-6) / 2.0
    print(f"  {name:<28} 首峰 {peak:5.1f}°  "
          f"衰减到<1.0°: {settle(1.0):4.2f}s  <0.5°: {settle(0.5):4.2f}s  "
          f"振荡≈{f:4.2f}Hz  |iq|max={np.max(np.abs(iq)):.2f}A")
    return settle(0.5)


def theory(j, kp_deg, kd, pv, iv):
    """速度内环阻尼比（外环未闭合，只看内环）"""
    wn = math.sqrt(KT * iv / j)
    zeta = KT * pv * (kd + 1) / (2 * j * wn)
    return wn, zeta


if __name__ == "__main__":
    print("=" * 78)
    print("  M0 摆动仿真：初始偏移 6°，看包络怎么衰减")
    print("=" * 78)

    for j in (5e-5, 2e-4, 1e-3):
        wn, _ = theory(j, 0.2, 0, 0.02, 1.0)
        print(f"\n--- 假设转动惯量 J = {j:.0e} kg·m²  (速度内环 ωn = {wn:.1f} rad/s = {wn/6.283:.1f} Hz) ---")

        for kd in (0.0, 0.3, 0.6, 1.0):
            _, z = theory(j, 0.2, kd, 0.02, 1.0)
            a = simulate(j, kp_deg=0.2, kd=kd, pv=0.02, iv=1.0)
            report(f"现状 Pv=0.02 kd={kd:.1f} (ζ={z:.2f})", a)

        for pv in (0.06, 0.10, 0.20):
            _, z = theory(j, 0.2, 0.0, pv, 1.0)
            a = simulate(j, kp_deg=0.2, kd=0.0, pv=pv, iv=1.0)
            report(f"★新方案 Pv={pv:.2f} kd=0 (ζ={z:.2f})", a)

    print("\n" + "=" * 78)
    print("  编码器量化噪声 + 抖动下（模拟真实 AS5600）")
    print("=" * 78)
    j = 2e-4
    for pv, kd in ((0.02, 0.0), (0.02, 1.0), (0.06, 0.0), (0.10, 0.0)):
        a = simulate(j, kp_deg=0.2, kd=kd, pv=pv, iv=1.0, t_end=8.0,
                     noise_lsb=0.5, seed=7)
        report(f"Pv={pv:.2f} kd={kd:.1f} (+噪声)", a)
