# -*- coding: utf-8 -*-
"""
验证正弦扫描模式的数学正确性（纯浮点复刻固件实现，不依赖硬件）。

复刻对象：lib/MYFOC/foc_motor.cpp
  _updateSweep()      θ = c + a·sin(w·t + φ)，vff = a·w·cos(w·t + φ)
  startSweep()        参数校验 + internal 坐标换算
  startSweepAuto()    φ = asin((θ_now - c)/a) - w·t，使启动瞬间输出 = 当前角

坐标系回顾（这是前面折腾很久才定下来的）：
  对外 u = user_sign · v（v 是控制链路坐标）
  M0: user_sign = +1
  M1: user_sign = -1（编码器反装）
"""
import math

D2R = math.pi / 180.0
R2D = 180.0 / math.pi

ok_all = True


def check(name, cond, detail=""):
    global ok_all
    ok_all = ok_all and bool(cond)
    print("  [%s] %s%s" % ("PASS" if cond else "FAIL", name,
                           ("  " + detail) if detail else ""))


class Sweep:
    def __init__(self, user_sign, lo_u, hi_u):
        self.s = user_sign
        self.lo_u = lo_u      # 对外坐标限位（度）
        self.hi_u = hi_u
        self.on = False
        # internal 限位（由 setUserSign 镜像而来）：lo_i = s·?, 这里按固件语义
        # 固件里 _pos_lo/_pos_hi 是 internal；u = s·v ⇒ v = s·u
        self.lo_i = min(user_sign * lo_u, user_sign * hi_u) * D2R
        self.hi_i = max(user_sign * lo_u, user_sign * hi_u) * D2R
        self.c = self.a = self.w = self.ph = self.vff = 0.0

    # ---- 对照固件 startSweep ----
    def start(self, center_deg, amp_deg, freq, phase_deg=None):
        a = abs(amp_deg)
        if a < 0.01:
            self.on = False
            return True
        if freq <= 0:
            freq = 0.3
        if freq > 5.0:
            freq = 5.0
        if center_deg - a < self.lo_u - 0.05 or center_deg + a > self.hi_u + 0.05:
            return False
        self.c = self.s * center_deg * D2R
        self.a = self.s * a * D2R
        self.w = 2.0 * math.pi * freq
        self.on = True
        if phase_deg is None:
            return "auto"
        self.ph = phase_deg * D2R
        return True

    # ---- 对照固件 startSweepAuto ----
    def start_auto(self, center_deg, amp_deg, freq, now_rad, t):
        r = self.start(center_deg, amp_deg, freq, None)
        if r is not True and r != "auto":
            return False
        if abs(self.a) > 1e-6:
            x = (now_rad - self.c) / self.a
            x = max(-1.0, min(1.0, x))
        else:
            x = 0.0
        self.ph = math.asin(x) - self.w * t
        return True

    def step(self, t):
        ang = self.w * t + self.ph
        tgt = self.c + self.a * math.sin(ang)
        vff = self.a * self.w * math.cos(ang)
        if tgt < self.lo_i:
            tgt, vff = self.lo_i, 0.0
        if tgt > self.hi_i:
            tgt, vff = self.hi_i, 0.0
        self.vff = vff
        return tgt

    def user_of(self, internal):
        return self.s * internal * R2D


print("=" * 70)
print("场景 1：M0（user_sign=+1）45~135°，中心90 幅度45")
print("=" * 70)
m0 = Sweep(+1, 0, 180)
r = m0.start(90, 45, 0.3, 0.0)
check("启动成功", r is True)
check("internal 中心 = +90°", abs(m0.c * R2D - 90) < 1e-6, "c=%.3f°" % (m0.c * R2D))
check("internal 幅度 = +45°", abs(m0.a * R2D - 45) < 1e-6)
# 波峰/波谷
tq = 0.25 / 0.3   # sin 在 ang=π/2 时 =1 → w·t = π/2
peak = m0.user_of(m0.step(tq))
trough = m0.user_of(m0.step(tq + 1.0 / 0.3 / 2))
check("波峰 = 135°", abs(peak - 135) < 0.01, "%.3f°" % peak)
check("波谷 = 45°", abs(trough - 45) < 0.01, "%.3f°" % trough)
# 峰值前馈速度
m0.step(0.0)   # sin0=0 → cos0=1 → vff = a·w
vpk_expected = 45 * D2R * (2 * math.pi * 0.3)
check("t=0 处前馈 = a·w", abs(m0.vff - vpk_expected) < 1e-6,
      "%.4f rad/s (%.0f°/s)" % (m0.vff, m0.vff * R2D))
# 周期内采样不越界
inside = True
for i in range(2001):
    t = i * (1.0 / 0.3) / 2000
    u = m0.user_of(m0.step(t))
    if u < 45 - 0.01 or u > 135 + 0.01:
        inside = False
check("一个周期内始终在 45~135°", inside)

print()
print("=" * 70)
print("场景 2：M1（user_sign=-1，编码器反装）45~135°")
print("=" * 70)
m1 = Sweep(-1, 0, 180)
r = m1.start(90, 45, 0.3, 0.0)
check("启动成功", r is True)
check("internal 中心 = -90°（镜像）", abs(m1.c * R2D - (-90)) < 1e-6, "c=%.3f°" % (m1.c * R2D))
check("internal 幅度 = -45°（符号随 user_sign）", abs(m1.a * R2D - (-45)) < 1e-6)
peak_u = m1.user_of(m1.step(tq))
check("**物理波峰仍是 135°**（符号正确抵消）", abs(peak_u - 135) < 0.01, "%.3f°" % peak_u)
# 前馈方向不能取在 cos=0 的波峰（那里恒为 0，浮点符号随机），改在 t=0 处测：
#   t=0 → ang=0 → cos=1 → internal vff = a·w，a<0 故为负；
#   而 internal 目标此时正朝负方向变化，两者同号才说明前馈方向对。
m1.step(0.0)
vff_u = m1.s * m1.vff
check("前馈在用户视角为正（t=0 处）", vff_u > 1e-3, "%.0f°/s" % (vff_u * R2D))

# ★ 最本质的检查：前馈必须真的等于 dθ/dt（数值微分对照），
#   而且 internal 与用户的符号关系要一致 —— 这才是"跟踪感"成立的前提。
print()
print("  前馈 vs 数值微分（internal 坐标，dt=1ms）:")
for sign, name in ((+1, "M0"), (-1, "M1")):
    sw = Sweep(sign, 0, 180)
    sw.start(90, 45, 0.4, 0.0)
    h = 1e-4
    worst = 0.0
    for i in range(1, 400):
        t = i * 0.001
        sw.step(t)
        v_ff = sw.vff
        dtheta = (sw.step(t + h) - sw.step(t - h)) / (2 * h)
        worst = max(worst, abs(v_ff - dtheta))
    check("%s 前馈 = dθ/dt（最大偏差 %.2e rad/s）" % (name, worst), worst < 1e-5)

print()
print("=" * 70)
print("场景 3：相位自动对齐（启动不跳变）")
print("=" * 70)
for sign, name in ((+1, "M0"), (-1, "M1")):
    sw = Sweep(sign, 0, 180)
    t_start = 12345.678          # 任意 millis 时刻
    # 假设当前物理角 60°，转成 internal
    now_u = 60.0
    now_i = sign * now_u * D2R
    sw.start_auto(90, 45, 0.3, now_i, t_start)
    first = sw.user_of(sw.step(t_start))
    check("%s 对齐后首拍输出 = 当前 60°" % name, abs(first - 60) < 1e-3,
          "%.3f°" % first)
    # 起步方向应向上（服务第一个周期内会超过 60°）
    later = sw.user_of(sw.step(t_start + 0.5))
    check("%s 起步向上（0.5s 后 > 60°）" % name, later > 60.0, "%.1f°" % later)

print()
print("=" * 70)
print("场景 4：限位校验（超范围必须拒绝而不是钳位）")
print("=" * 70)
s = Sweep(+1, 0, 180)
check("90±45 通过", s.start(90, 45, 0.3, 0) is True)
s2 = Sweep(+1, 0, 180)
check("90±91 拒绝（超出 180）", s2.start(90, 91, 0.3, 0) is False)
s3 = Sweep(+1, 0, 180)
check("90±90 通过（正好贴边）", s3.start(90, 90, 0.3, 0) is True)
s4 = Sweep(+1, 0, 180)
check("中心 10 幅度 20 拒绝（下限越界）", s4.start(10, 20, 0.3, 0) is False)
s5 = Sweep(+1, 45, 135)    # 用户把限位收紧到 45~135
check("限位 45~135 下 90±45 通过", s5.start(90, 45, 0.3, 0) is True)
s6 = Sweep(+1, 45, 135)
check("限位 45~135 下 90±50 拒绝", s6.start(90, 50, 0.3, 0) is False)

print()
print("=" * 70)
print("场景 5：频率 / 峰值速度 / 跟踪滞后估算")
print("=" * 70)
print("  幅度45°（45~135）:")
print("    f(Hz)  周期(s)  单程(s)   峰值速度(deg/s)   位置环稳态滞后(deg)")
KP_EFF = 0.2 * 57.2958     # 位置环 P=0.2（度为单位）→ 等效 11.46 /s
for f in (0.2, 0.3, 0.4, 0.5, 0.7, 1.0):
    vpk = 45 * D2R * 2 * math.pi * f          # rad/s
    lag = vpk / KP_EFF * R2D                  # 无前馈时的稳态滞后
    print("    %.1f    %.2f     %.2f      %6.0f            %5.1f%s"
          % (f, 1.0 / f, 0.5 / f, vpk * R2D, lag,
             "   <- 前馈基本消除" if f <= 0.5 else ""))

print()
print("=" * 70)
print("总结：%s" % ("全部通过" if ok_all else "有失败项，需修正"))
print("=" * 70)
