#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""对比表：旧参数(Pv=0.02) vs 新参数(Pv=0.15) 的摆动包络"""
import math
import numpy as np
from sim_damping import simulate

DT = 0.0037


def envelope(arr, t_marks):
    t, th = arr[:, 0], arr[:, 1]
    out = []
    for tm in t_marks:
        m = (t >= tm) & (t < tm + 0.5)
        out.append(np.max(np.abs(th[m])) if np.any(m) else 0.0)
    return out


def last_above(arr, thr):
    t, th = arr[:, 0], arr[:, 1]
    idx = np.where(np.abs(th) > thr)[0]
    return t[idx[-1]] if len(idx) else 0.0


def run(j, pv, kd, label):
    a = simulate(j, kp_deg=0.2, kd=kd, pv=pv, iv=1.0,
                 theta0_deg=6.0, t_end=6.0, noise_lsb=0.3, seed=3)
    marks = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0]
    env = envelope(a, marks)
    wn = math.sqrt(0.10 * 1.0 / j)
    zeta = 0.10 * pv * (kd + 1) / (2 * j * wn)
    print(f"{label:<26} ζ={zeta:4.2f}  " +
          "  ".join(f"{v:5.2f}" for v in env) +
          f"   停住(|θ|<0.3°)时刻 {last_above(a, 0.3):4.2f}s")
    return a, env


if __name__ == "__main__":
    print("初始扰动 6°，每 0.5s 窗口内的最大摆幅（度）\n")
    for j, jn in ((2e-4, "J=2e-4"), (5e-4, "J=5e-4"), (1e-3, "J=1e-3")):
        print(f"=== {jn} kg·m² ===")
        print(f"{'参数':<26} {'':<6} " + "  ".join(f" {m:.1f}s" for m in (0, 1, 2, 3, 4, 5)))
        run(j, 0.02, 0.3, "旧 Pv=0.02 kd=0.3")
        run(j, 0.02, 1.0, "旧 Pv=0.02 kd=1.0")
        run(j, 0.15, 0.3, "★新 Pv=0.15 kd=0.3")
        run(j, 0.25, 0.3, "  新 Pv=0.25 kd=0.3")
        print()
