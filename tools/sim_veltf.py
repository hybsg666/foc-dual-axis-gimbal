#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
扫速度反馈低通时间常数 Tf：既要 M0 不摆，又要电流纹波足够小（= 不吵）

背景：速度环 P 从 0.02 提到 0.15 后，AS5600 的测速噪声（量化 0.088°/3.7ms
≈ 0.4 rad/s）被放大 7.5 倍 → iq 纹波 → 电磁噪声。
低通会把噪声压下去，但也会给阻尼带来相位滞后。
这里同时看两个指标：
  1) 停住时间（阻尼是否还在）
  2) iq 高频纹波 RMS（≈ 噪音大小）
"""
import math
import numpy as np
from sim_damping import simulate


def analyse(j, pv, tf, noise_lsb=0.5, seed=5):
    a = simulate(j, kp_deg=0.2, kd=0.3, pv=pv, iv=1.0,
                 theta0_deg=6.0, t_end=5.0, noise_lsb=noise_lsb, seed=seed, tf_vel=tf)
    t, th, om, iq = a.T

    # 停住时间：最后一次 |θ|>0.3°
    idx = np.where(np.abs(th) > 0.3)[0]
    settle = t[idx[-1]] if len(idx) else 0.0

    # 稳态段（3~5s）的 iq 高频纹波：先一阶差分去慢变，再取 RMS
    m = t > 3.0
    d = np.diff(iq[m])
    ripple = float(np.sqrt(np.mean(d ** 2)))

    # 稳态角度抖动
    jitter = float(np.std(th[m]))
    return settle, ripple, jitter


if __name__ == "__main__":
    print("速度环 P=0.15 固定，扫速度反馈低通 Tf")
    print("（M0 场景：J=1e-3，有编码器量化抖动噪声）\n")
    print(f"{'Tf (s)':>8}  {'停住时间':>9}  {'iq逐拍变化RMS':>14}  {'角度抖动σ':>10}   评价")
    print("-" * 68)

    base = None
    for tf in (0.005, 0.01, 0.015, 0.02, 0.025, 0.03, 0.04):
        s, r, jt = analyse(1e-3, 0.15, tf)
        if base is None:
            base = r
        rel = r / base
        if s > 1.0:
            verdict = "还在摆，不能用"
        elif rel > 0.8:
            verdict = "降噪不明显"
        elif rel < 0.35:
            verdict = "★ 推荐"
        else:
            verdict = "可用"
        print(f"{tf:8.3f}  {s:8.2f}s  {r:14.5f}  {jt:9.3f}°   {verdict} (噪声×{rel:.2f})")

    print("\n对照：M1 场景（小惯量 J=2e-5），不同速度环 P 的噪声")
    print(f"{'Pv':>6}  {'Tf':>6}  {'停住时间':>9}  {'iq纹波RMS':>12}   评价")
    print("-" * 55)
    for pv in (0.02, 0.05, 0.10, 0.15):
        for tf in (0.025,):
            s, r, jt = analyse(2e-5, pv, tf)
            v = "安静" if r < 0.004 else ("偏吵" if r < 0.012 else "很吵")
            print(f"{pv:6.2f}  {tf:6.3f}  {s:8.2f}s  {r:12.5f}   {v}")
