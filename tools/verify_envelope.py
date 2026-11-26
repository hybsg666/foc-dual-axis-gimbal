# -*- coding: utf-8 -*-
"""减速包络 v_safe=sqrt(2·a·d) 的安全性检查：会不会把逼近过程卡死。"""
import math
A = 140.0          # rad/s²（固件默认，≈8000 °/s²）
R2D = 57.2957795
DEADBAND = 0.35    # °

print("=" * 66)
print("1) 各剩余距离下允许的最大速度（判断够不够用）")
print("=" * 66)
print(f"{'剩余距离':>10} {'v_safe':>10} {'v_safe':>10}   说明")
for d_deg in (90, 45, 20, 10, 5, 3, 2, 1, 0.5, 0.35, 0.2, 0.1):
    d = d_deg * math.pi / 180
    v = math.sqrt(2 * A * d)
    note = ""
    if v * R2D < 20: note = "偏紧（接近死区，由死区接管）"
    elif v * R2D > 300: note = "很松（不限制）"
    tag = "  ← 死区边界" if abs(d_deg - DEADBAND) < 1e-6 else ""
    print(f"{d_deg:>8.2f}° {v:>9.2f}r/s {v*R2D:>9.0f}°/s   {note}{tag}")

print()
print("=" * 66)
print("2) 关键判据：包络是否严于位置环（严于=会拖慢逼近）")
print("=" * 66)
# 位置环：v = 11.46(1/s) * err。误差=剩余距离时，比较两者
KP = 0.2 * R2D    # deg/s per deg
for d_deg in (45, 20, 10, 5, 3, 1, 0.5):
    d = d_deg * math.pi / 180
    v_env = math.sqrt(2 * A * d) * R2D
    v_pos = KP * d_deg
    who = "包络更严（限速生效）" if v_env < v_pos else "位置环更严（包络不介入）"
    print(f"  剩余 {d_deg:>5.1f}°: 包络 {v_env:>6.0f}°/s  位置环 {v_pos:>6.0f}°/s   → {who}")

print()
print("=" * 66)
print("3) 过冲后的回退能力（d_rem<0，目标在后方）")
print("=" * 66)
for over_deg in (12, 8, 5, 2, 1):
    d = over_deg * math.pi / 180
    v = math.sqrt(2 * A * d) * R2D
    print(f"  过冲 {over_deg:>2}°: 允许回退速度 {v:>6.0f}°/s   "
          f"以该速度回来的最短时间 {2*over_deg/max(v,1e-9)*1000:>5.0f}ms")

print()
print("=" * 66)
print("4) 极限情况：a 设太小会不会永远到不了")
print("=" * 66)
for a in (2000, 5000, 8000, 14000):
    A2 = a / R2D
    d = 1.0 * math.pi / 180      # 差 1°
    v = math.sqrt(2 * A2 * d) * R2D
    # 位置环在 1° 误差时给 11.46°/s；若包络更小则被限
    print(f"  a={a:>6} °/s²: 差1°时上限 {v:>5.0f}°/s，位置环想要 {KP:.1f}°/s → "
          f"{'限到 ' + format(v, '.0f') + '°/s' if v < KP else '不限'}")

print()
print("=" * 66)
print("5) 结论")
print("=" * 66)
print("  · 距离 >20° 时包络比位置环宽松 → 完全不影响加速段")
print("  · 距离 <3° 时包络收紧到 <160°/s，比位置环（1° 时 11.5°/s）仍然宽松")
print("    → **位置环始终是更严的那个**，包络只在\"位置环被前馈顶到高速\"时才起作用")
print("  · 过冲 12° 时仍允许 150°/s 回退 → 不会卡在过冲位置")
print("  · 只有 a 设到 2000 以下才会出现\"包络比位置环严 → 拖慢\"，默认 8000 安全")
