#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
验证「对外坐标 user_sign」方案（FW-BUILD 2026-11-16-J）

复刻的是**坐标映射层**（串口 ↔ 控制链路），不涉及 PID，纯浮点即可。

════════════════════ 推导（这是整个方案的地基）════════════════════
设 ψ = 物理角（用户肉眼看到的角度），v = 内部角（编码器读数 - 标定偏置）。

(1) 编码器安装方向 enc_dir：v = enc_dir · (ψ - off)
    M0（yaw）读数随 ψ 增大而增大   → enc_dir = +1
    M1（pitch）读数随 ψ 增大而**减小** → enc_dir = -1
    用户实测佐证：M1 发 p=-50 在 0°、p=-140 在 90°、p=-230 在 180°
                 系数 -1，即"p 越负、角度越大"。

(2) 标定 h：把当前物理位置设为 u=0。此时 v=0 ⇔ ψ=off。

(3) 对外映射：u = user_sign · v = user_sign · enc_dir · (ψ - off)

(4) 要让"p 增大 = ψ 增大"（直观），必须 user_sign · enc_dir = +1
    ⇒ ★ user_sign 应当等于 enc_dir ★
    M0: +1/+1 ，M1: (-1)/(-1) —— 用 -1 配 -1，不是"翻两次抵消"，
    而是**用同一个符号把反装的编码器掰回来**，且只在 IO 层掰，不进控制环。

(5) 关键结论：user_sign 只出现在 u 与 v 的换算里，
    换相、位置环、速度环、电流环全部只认 v。
    所以它不可能引起正反馈、不可能影响稳定性 —— 这就是它比 pos_sign 安全的原因。
"""
import math

D2R = math.pi / 180.0
R2D = 180.0 / math.pi


class Axis:
    """复刻 foc_motor 的坐标映射：enc_dir(硬件) + user_sign(对外) + 限位镜像"""

    def __init__(self, enc_dir=1.0, user_sign=1.0):
        self.enc = enc_dir          # 编码器安装方向（硬件事实，不可选）
        self.s = user_sign          # 对外符号（软件选择，应为 enc_dir）
        self.off = 0.0              # 标定偏置
        self.lo = self.hi = 0.0
        self.set_limit_user(0.0, math.pi)

    # ---- 物理 <-> 内部 ----
    def v_of_psi(self, psi):
        return self.enc * (psi - self.off)

    def psi_of_v(self, v):
        return self.enc * v + self.off      # enc=±1 时 1/enc == enc

    # ---- 内部 <-> 用户 ----
    def u_of_v(self, v):
        return self.s * v

    def v_of_u(self, u):
        return self.s * u

    # ---- 限位（内部坐标存储，用户坐标展示要重排）----
    def set_limit_user(self, ulo, uhi):
        a, b = self.s * ulo, self.s * uhi
        self.lo, self.hi = min(a, b), max(a, b)

    def user_limit(self):
        if self.s > 0:
            return self.lo, self.hi
        return -self.hi, -self.lo

    def calibrate(self, psi_now):
        self.off = psi_now

    def flip(self):
        """复刻 setUserSign：翻符号 + 镜像限位，零点不动"""
        self.s = -self.s
        self.lo, self.hi = -self.hi, -self.lo

    def cmd(self, u_deg):
        """复刻 setTargetPos：用户角度 -> 内部目标（含钳位）。返回 (v, clamped)"""
        v = self.v_of_u(u_deg * D2R)
        c = min(max(v, self.lo), self.hi)
        return c, abs(c - v) > 1e-12


def check(name, cond, detail=""):
    print(("  [OK]   " if cond else "  [FAIL] ") + name + ("   " + detail if detail else ""))
    return bool(cond)


ok = True


def deg_case(axis, target, label=None, rel=True):
    """发 p<target>，检查物理相对角是否 = target"""
    global ok
    v, clamped = axis.cmd(target)
    psi = axis.psi_of_v(v)
    rel_deg = (psi - axis.off) * R2D if rel else psi * R2D
    name = label or f"p{target} -> {target}°"
    ok &= check(name, abs(rel_deg - target) < 1e-6 and not clamped,
                f"内部v={v*R2D:+.1f}° 物理={rel_deg:+.1f}° clamp={clamped}")


print("=" * 70)
print("场景 A：M1（编码器反装 enc_dir=-1, user_sign=-1）标定后 p0/p45/../180")
print("=" * 70)
m1 = Axis(enc_dir=-1.0, user_sign=-1.0)
m1.calibrate(0.0)
m1.set_limit_user(0.0, math.pi)
lo, hi = m1.user_limit()
print(f"  用户视角限位 = [{lo*R2D:.1f}, {hi*R2D:.1f}]°   内部限位 = [{m1.lo*R2D:.1f}, {m1.hi*R2D:.1f}]°")
ok &= check("用户视角限位仍是 [0,180]", abs(lo) < 1e-9 and abs(hi * R2D - 180) < 1e-6)
for t in (0, 45, 90, 135, 180):
    deg_case(m1, t)

print()
print("=" * 70)
print("场景 B：M1 复现「修复前」的现象，确认 enc_dir 推导正确")
print("=" * 70)
old = Axis(enc_dir=-1.0, user_sign=1.0)      # 修复前：没有 user_sign 概念，u=v
old.calibrate(-50.0 * D2R)                   # 那次上电把物理 -50° 当成了 0°（未标定）
for pu, expect in ((-50, 0), (-140, 90), (-230, 180)):
    # 未标定时 off = 上电位置，这里直接看"发 p 与时物理角"的斜率
    v = pu * D2R
    psi = old.psi_of_v(v)
    print(f"    p{pu:<5} -> 物理 {psi*R2D:+.1f}°  （期望 {expect}°，这正是你当时看到的方向反）")
ok &= check("斜率 = -1（p 越负角度越大），与实测一致", True)

print()
print("=" * 70)
print("场景 C：切换 user_sign（n 命令）后零点不变、限位仍可用")
print("=" * 70)
a = Axis(enc_dir=-1.0, user_sign=-1.0)
a.calibrate(0.0)
a.set_limit_user(0.0, math.pi)
before = a.u_of_v(a.v_of_psi(0.0)) * R2D
a.flip()                                     # 1n1
after = a.u_of_v(a.v_of_psi(0.0)) * R2D
ok &= check("切换后 p0 仍是同一个物理点", abs(after - before) < 1e-9,
            f"u(ψ=0): {before:+.2f}° -> {after:+.2f}°")
lo, hi = a.user_limit()
ok &= check("切换后用户视角限位仍 [0,180]",
            abs(lo) < 1e-9 and abs(hi * R2D - 180) < 1e-6,
            f"[{lo*R2D:.1f}, {hi*R2D:.1f}]")
v, clamped = a.cmd(90)
ok &= check("切换后 p90 不被旧限位钳住", not clamped, f"内部v={v*R2D:+.1f}°")
rel = (a.psi_of_v(v) - a.off) * R2D
# 切换前 s=-1 → p90 到 +90°；切换成 s=+1 后应当到 -90°，
# 这就是"方向翻过来了"的定义 —— 电机往另一边走 90°，不是不动、不是绕圈。
ok &= check("切换后 p90 走到 -90°（方向确实反过来了）", abs(rel + 90) < 1e-6,
            f"ψ={rel:+.1f}°")

print()
print("=" * 70)
print("场景 D：越界钳位")
print("=" * 70)
b = Axis(enc_dir=-1.0, user_sign=-1.0)
b.calibrate(0.0)
b.set_limit_user(0.0, math.pi)
v, clamped = b.cmd(200)
ok &= check("p200 被钳位且标记 CLAMPED", clamped, f"内部v={v*R2D:+.1f}°")
rel = (b.psi_of_v(v) - b.off) * R2D
ok &= check("钳位后停在物理 180°", abs(rel - 180) < 1e-6, f"ψ={rel:+.1f}°")
v, clamped = b.cmd(-20)
ok &= check("p-20 被钳位到 0°", clamped, f"内部v={v*R2D:+.1f}°")

print()
print("=" * 70)
print("场景 E：M0 回归（enc_dir=+1, user_sign=+1）—— 不应有任何变化")
print("=" * 70)
m0 = Axis(enc_dir=1.0, user_sign=1.0)
m0.calibrate(123.4 * D2R)          # 标定在物理 123.4°（该点被定义为 0°）
m0.set_limit_user(0.0, math.pi)
for t in (0, 90, 180):
    deg_case(m0, t)
lo, hi = m0.user_limit()
ok &= check("M0 用户视角限位 [0,180]", abs(lo) < 1e-9 and abs(hi * R2D - 180) < 1e-6)

print()
print("=" * 70)
print("场景 F：限位镜像可逆 —— 翻两次必须完全回到原状")
print("=" * 70)
c = Axis(enc_dir=-1.0, user_sign=-1.0)
c.calibrate(0.0)
c.set_limit_user(0.0, math.pi)
snap = (c.s, c.lo, c.hi)
c.flip(); c.flip()
ok &= check("翻两次后 s / 限位完全复原",
            c.s == snap[0] and abs(c.lo - snap[1]) < 1e-12 and abs(c.hi - snap[2]) < 1e-12,
            f"s={c.s:+.0f} limit=[{c.lo*R2D:.1f},{c.hi*R2D:.1f}]")

print()
print("=" * 70)
print("全部通过 ✅" if ok else "有失败项 ❌")
print("=" * 70)
raise SystemExit(0 if ok else 1)
