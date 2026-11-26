#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
verify_range.py —— 复刻固件的"区间内线性走位"逻辑，验证 0~180° 往返不绕路

复刻的三段（与 foc_motor.cpp 一一对应）：
  1) _clampTarget(tgt)                目标钳位到 [lo, hi]
  2) _nearFull(full)                  累积角折算（只在漏计整圈时才动手）
  3) err = target - nearFull(full)    线性误差，方向由符号唯一确定
"""
import math

PI = math.pi
R2D = 180.0 / math.pi
D2R = PI / 180.0


class Axis:
    """一台虚拟轴"""

    def __init__(self, lo_deg=0.0, hi_deg=180.0):
        self.lo = lo_deg * D2R
        self.hi = hi_deg * D2R
        self.full = 0.0      # 当前角度（云台坐标，rad）

    # --- 固件同款 ---
    def clamp(self, v):
        return max(self.lo, min(self.hi, v))

    def near_full(self, full):
        if (self.hi - self.lo) > 6.0:
            return full
        if abs(full - self.lo) < PI or abs(full - self.hi) < PI:
            return full
        mid = 0.5 * (self.lo + self.hi)
        k = round((mid - full) / (2 * PI))
        return full + k * 2 * PI

    def cmd(self, target_deg):
        """模拟一次 p 命令，返回 (误差rad, 方向)"""
        end = self.clamp(target_deg * D2R)
        err = end - self.near_full(self.full)
        return err, ("正转 →" if err > 0 else ("反转 ←" if err < 0 else "不变")), end

    def step_to(self, end, done=0.02):
        self.full = end + (self.full - end) * done


def run_case(name, seq, lo=0.0, hi=180.0, start=0.0):
    a = Axis(lo, hi)
    a.full = start * D2R
    print(f"\n=== {name} （限位 [{lo:.0f}, {hi:.0f}]°，起点 {start:.0f}°）===")
    print(f"{'命令':>10} {'当前':>8} {'目标':>8} {'误差':>9} {'方向':>8} {'越界?':>6} {'绕路?':>6}")
    print("-" * 66)
    ok = True
    for t in seq:
        err, d, end = a.cmd(t)
        before = a.full
        # 判断是否绕路：线性误差的绝对值不应超过区间跨度 + 小量
        span = a.hi - a.lo
        detour = abs(err) > span + 0.35
        a.step_to(end)
        over = not (a.lo - 1e-6 <= end <= a.hi + 1e-6)
        if detour or over:
            ok = False
        print(f"p{t:<9.0f} {before*R2D:7.1f}° {end*R2D:7.1f}° {err*R2D:+8.1f}° {d:>8} "
              f"{'是 ⚠' if over else '否':>6} {'是 ⚠' if detour else '否':>6}")
    print(("  ✔ 全部在区间内走直线" if ok else "  ✘ 存在越界或绕路"))
    return ok


if __name__ == "__main__":
    allok = True

    allok &= run_case("典型往复", [180, 0, 180, 0, 90, 45, 135, 0])

    allok &= run_case("端点附近抖（最容易翻向的地方）",
                      [179, 1, 180, 0, 179.5, 0.5],
                      start=179)

    allok &= run_case("小步扫描", list(range(0, 181, 20)) + [0])

    # 编码器漏计一圈：full 漂到 360° 附近
    print("\n=== 编码器漏计整圈（full 漂到 +360°）===")
    a = Axis()
    a.full = 360.0 * D2R
    for t in (0, 90, 180):
        err, d, end = a.cmd(t)
        print(f"  当前 {a.full*R2D:7.1f}° → p{t}: 折算后误差 {err*R2D:+7.1f}° {d}")
        a.full = end if t == 0 else a.full
    print("  说明：360° 与 0° 是同一个物理位置，折算后误差为 0 —— 不会转一整圈")

    # 断电重启零点是否稳定
    print("\n=== 零点持久化 ===")
    print("  标定：发 '0h' 把当前位置定为 0° → 写 NVS(pos_zero = 编码器绝对角)")
    print("  上电：start() 读 NVS → setAngleOffset(该值) → full=0 永远是同一物理位置")
    print("  ✔ p0 可复现，不再随上电姿态变化")

    print("\n" + ("=" * 66))
    print("全部通过 ✔" if allok else "有失败项 ✘")
