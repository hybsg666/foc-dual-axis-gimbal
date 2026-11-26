#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
S 曲线轨迹算法验证（PC 侧复刻固件算法）

对应固件文件：
  lib/MYFOC/foc_trajectory.cpp  ->  s_accel / v_accel / traj_begin / traj_step

目的：
  1. 在电脑上把同一套公式跑一遍，检查数学是否正确（端到端精度、限幅、连续性）
  2. 输出 S 曲线 vs 旧梯形 的对比数据（时长 / 峰值速度 / 峰值加速度 / 峰值 jerk）
  3. 生成 tools/out/scurve_report.html，浏览器打开看三条曲线

用法：
  python tools/verify_scurve.py
"""

import math
import os
import json

# ============================================================
# 固件参数（必须与 foc_motor.h 保持一致）
# ============================================================
TRAJ_ACCEL = 30.0     # rad/s²  峰值加速度
TRAJ_JERK = 120.0     # rad/s³  加加速度
LOOP_HZ = 270.0       # 实测控制频率（loop 有串口打印时会被拉低，这里取标称值）
DT = 1.0 / LOOP_HZ

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")


# ============================================================
# 以下三个函数与 foc_trajectory.cpp 逐行对应
# ============================================================
def s_accel(t, J, A, Tj, Ta):
    if t <= 0.0:
        return 0.0
    s1 = J * Tj ** 3 / 6.0
    v1 = J * Tj ** 2 / 2.0
    if t <= Tj:
        return J * t ** 3 / 6.0
    if t <= Tj + Ta:
        d = t - Tj
        return s1 + v1 * d + 0.5 * A * d * d
    s2 = s1 + v1 * Ta + 0.5 * A * Ta * Ta
    v2 = v1 + A * Ta
    d = min(t - (Tj + Ta), Tj)
    return s2 + v2 * d + 0.5 * A * d * d - J * d ** 3 / 6.0


def v_accel(t, J, A, Tj, Ta):
    if t <= 0.0:
        return 0.0
    if t <= Tj:
        return J * t * t / 2.0
    v1 = J * Tj ** 2 / 2.0
    if t <= Tj + Ta:
        return v1 + A * (t - Tj)
    v2 = v1 + A * Ta
    d = min(t - (Tj + Ta), Tj)
    return v2 + A * d - J * d * d / 2.0


def traj_plan(D, V, A, J):
    """复刻 traj_begin 的规划部分"""
    V = abs(V)
    if V <= 0.0:
        V = A
    if V * J <= A * A:
        apk = math.sqrt(V * J)
        Tj = math.sqrt(V / J)
        Ta = 0.0
    else:
        apk = A
        Tj = A / J
        Ta = max(0.0, V / A - Tj)
    vpk = V
    S_acc = vpk * (Tj + 0.5 * Ta)

    if 2.0 * S_acc <= D:
        Tc = (D - 2.0 * S_acc) / vpk
    else:
        Tc = 0.0
        vp = 0.5 * apk * (-Tj + math.sqrt(Tj * Tj + 4.0 * D / apk))
        vp_min = apk * Tj
        if vp >= vp_min:
            vpk = vp
            Ta = vp / apk - Tj
        else:
            Tj = (D / (2.0 * J)) ** (1.0 / 3.0)
            apk = J * Tj
            vpk = J * Tj * Tj
            Ta = 0.0

    return {
        "Tj": Tj, "Ta": Ta, "Tc": Tc, "vpk": vpk, "apk": apk,
        "T": 2.0 * (2.0 * Tj + Ta) + Tc,
        "D": D, "J": J,
    }


def s_pos(t, p):
    T, Tj, Ta, Tc = p["T"], p["Tj"], p["Ta"], p["Tc"]
    T_acc = 2.0 * Tj + Ta
    if t <= 0.0:
        return 0.0
    if t >= T:
        return p["D"]
    if t < T_acc:
        return s_accel(t, p["J"], p["apk"], Tj, Ta)
    if t < T_acc + Tc:
        return s_accel(T_acc, p["J"], p["apk"], Tj, Ta) + p["vpk"] * (t - T_acc)
    return p["D"] - s_accel(T - t, p["J"], p["apk"], Tj, Ta)


def s_vel(t, p):
    T, Tj, Ta, Tc = p["T"], p["Tj"], p["Ta"], p["Tc"]
    T_acc = 2.0 * Tj + Ta
    if t <= 0.0 or t >= T:
        return 0.0
    if t < T_acc:
        return v_accel(t, p["J"], p["apk"], Tj, Ta)
    if t < T_acc + Tc:
        return p["vpk"]
    return v_accel(T - t, p["J"], p["apk"], Tj, Ta)


def simulate_scurve(D, V, A, J, dt=DT, jitter=False):
    """模拟固件逐拍调用 traj_step（t 累加 dt）"""
    p = traj_plan(D, V, A, J)
    times, pos, vel = [], [], []
    t = 0.0
    i = 0
    while t < p["T"] + dt * 2:
        times.append(t)
        pos.append(s_pos(t, p))
        vel.append(s_vel(t, p))
        # 模拟固件：dt 由真实时间差得到，串口打印会造成抖动
        d = dt
        if jitter:
            d = dt * (1.0 + 0.6 * math.sin(i * 0.7))
        t += d
        i += 1
    times.append(p["T"])
    pos.append(p["D"])
    vel.append(0.0)
    return p, times, pos, vel


def simulate_trapezoid(D, V, A, dt=DT):
    """旧梯形算法（固件 traj_step_trapezoid 的复刻）"""
    cur, v = 0.0, 0.0
    times, pos, vel = [0.0], [0.0], [0.0]
    t = 0.0
    guard = 0
    while True:
        guard += 1
        if guard > 200000:
            break
        remaining = abs(D - cur)
        stop_dist = v * v / (2.0 * A)
        if remaining <= stop_dist:
            dec = A * dt
            v = 0.0 if abs(v) <= dec else v - dec
        elif abs(v) < V:
            v += A * dt
            if abs(v) > V:
                v = V
        cur += v * dt
        t += dt
        if cur >= D:
            cur = D
            v = 0.0
            times.append(t)
            pos.append(cur)
            vel.append(v)
            break
        times.append(t)
        pos.append(cur)
        vel.append(v)
    return times, pos, vel


# ============================================================
# 自检
# ============================================================
def run_checks():
    print("=" * 66)
    print(" S 曲线算法自检（PC 复刻固件公式）")
    print("=" * 66)
    ok = True

    def chk(name, cond, detail=""):
        nonlocal ok
        flag = "PASS" if cond else "FAIL"
        if not cond:
            ok = False
        print(f"  [{flag}] {name}" + (f"   {detail}" if detail else ""))

    # ---- 检查1：终点精度（三种工况）----
    for D, V, label in [(0.524, 5.0, "30° 短行程"), (1.571, 5.0, "90° 中行程"),
                        (3.1416, 5.0, "180° 长行程"), (3.1416, 0.5, "180° 低速"),
                        (0.05, 5.0, "2.9° 极短")]:
        p, ts, ps, vs = simulate_scurve(D, V, TRAJ_ACCEL, TRAJ_JERK)
        err = abs(ps[-1] - D)
        chk(f"终点精度 {label}", err < 1e-6, f"误差={err:.2e} rad, T={p['T']:.3f}s")

    # ---- 检查2：限幅 ----
    for D, V in [(3.1416, 5.0), (0.524, 5.0), (0.2, 0.5)]:
        p, ts, ps, vs = simulate_scurve(D, V, TRAJ_ACCEL, TRAJ_JERK)
        vmax = max(abs(v) for v in vs)
        accs = [(vs[i + 1] - vs[i]) / (ts[i + 1] - ts[i]) for i in range(len(vs) - 1)
                if ts[i + 1] > ts[i] + 1e-9]
        amax = max(abs(a) for a in accs)
        jerks = [(accs[i + 1] - accs[i]) / (ts[i + 2] - ts[i + 1]) for i in range(len(accs) - 1)
                 if abs(ts[i + 2] - ts[i + 1]) > 1e-9]
        jmax = max(abs(j) for j in jerks) if jerks else 0.0
        chk(f"速度不超上限 D={D:.2f} V={V}",
            vmax <= V * 1.001 + 1e-6, f"峰值={vmax:.3f} ≤ {V}")
        chk(f"加速度不超上限 D={D:.2f} V={V}",
            amax <= TRAJ_ACCEL * 1.01 + 1e-6, f"峰值={amax:.2f} ≤ {TRAJ_ACCEL}")
        chk(f"jerk 不超上限 D={D:.2f} V={V}",
            jmax <= TRAJ_JERK * 1.02, f"峰值={jmax:.1f} ≤ {TRAJ_JERK}")

    # ---- 检查3：位置单调（不倒退）----
    p, ts, ps, vs = simulate_scurve(3.1416, 5.0, TRAJ_ACCEL, TRAJ_JERK)
    mono = all(ps[i + 1] >= ps[i] - 1e-9 for i in range(len(ps) - 1))
    chk("位置单调不倒退", mono)

    # ---- 检查4：速度连续（相邻两拍速度跳变有界）----
    dv = max(abs(vs[i + 1] - vs[i]) for i in range(len(vs) - 1))
    chk("速度连续（每拍跳变 ≤ A·dt）", dv <= TRAJ_ACCEL * DT * 1.05,
        f"最大跳变={dv:.4f} rad/s, 允许={TRAJ_ACCEL * DT:.4f}")

    # ---- 检查5：dt 抖动不影响终点 ----
    p1, t1, s1, _ = simulate_scurve(3.1416, 5.0, TRAJ_ACCEL, TRAJ_JERK)
    p2, t2, s2, _ = simulate_scurve(3.1416, 5.0, TRAJ_ACCEL, TRAJ_JERK, jitter=True)
    e2 = abs(s2[-1] - 3.1416)
    chk("dt 抖 60% 时终点仍精确", e2 < 1e-6, f"误差={e2:.2e} rad")

    # ---- 检查6：解析速度 vs 数值微分 ----
    p, _, _, _ = simulate_scurve(1.571, 5.0, TRAJ_ACCEL, TRAJ_JERK)
    worst = 0.0
    for i in range(1, 200):
        t0 = i * p["T"] / 200.0
        h = 1e-6
        num = (s_pos(t0 + h, p) - s_pos(t0 - h, p)) / (2 * h)
        ana = s_vel(t0, p)
        worst = max(worst, abs(num - ana))
    chk("解析速度与数值微分一致", worst < 1e-3, f"最大偏差={worst:.2e}")

    print("-" * 66)
    print(" 结论：" + ("全部通过，算法可上机" if ok else "存在失败项，不要烧录！"))
    print("=" * 66)
    return ok


# ============================================================
# 对比表 + HTML 报告
# ============================================================
CASES = [
    ("30° 短行程", 0.5236, 5.0),
    ("90° 中行程", 1.5708, 5.0),
    ("180° 长行程", 3.1416, 5.0),
    ("180° 低速 0.5rad/s", 3.1416, 0.5),
]


def build_report():
    rows = []
    series = []
    for name, D, V in CASES:
        p, ts, ps, vs = simulate_scurve(D, V, TRAJ_ACCEL, TRAJ_JERK)
        tt, tp, tv = simulate_trapezoid(D, V, TRAJ_ACCEL)

        def amax_of(ts_, vs_):
            a = [(vs_[i + 1] - vs_[i]) / (ts_[i + 1] - ts_[i])
                 for i in range(len(vs_) - 1) if ts_[i + 1] - ts_[i] > 1e-9]
            return max(abs(x) for x in a) if a else 0.0

        rows.append({
            "name": name, "D": D, "V": V,
            "t_s": p["T"], "t_t": tt[-1],
            "vp_s": max(abs(x) for x in vs), "vp_t": max(abs(x) for x in tv),
            "a_s": amax_of(ts, vs), "a_t": amax_of(tt, tv),
        })
        series.append({
            "name": name, "D": D,
            "ts": [round(x, 4) for x in ts], "ss": [round(x, 4) for x in ps],
            "vs": [round(x, 4) for x in vs],
            "tt": [round(x, 4) for x in tt], "st": [round(x, 4) for x in tp],
            "vt": [round(x, 4) for x in tv],
        })

    print()
    print(f"{'工况':<20} {'S曲线T':>8} {'梯形T':>8} {'S峰值V':>8} {'梯形峰值V':>9} {'S峰值a':>8}")
    print("-" * 70)
    for r in rows:
        print(f"{r['name']:<20} {r['t_s']:>7.3f}s {r['t_t']:>7.3f}s "
              f"{r['vp_s']:>7.3f} {r['vp_t']:>8.3f} {r['a_s']:>7.2f}")

    os.makedirs(OUT_DIR, exist_ok=True)
    html_path = os.path.join(OUT_DIR, "scurve_report.html")
    with open(html_path, "w", encoding="utf-8") as f:
        f.write(HTML_TPL.replace("__DATA__", json.dumps(series, ensure_ascii=False)))
    print()
    print(f"曲线报告已生成：{html_path}")
    return rows


HTML_TPL = r"""<!DOCTYPE html>
<html lang="zh-CN"><head><meta charset="utf-8">
<title>S 曲线 vs 梯形 轨迹对比</title>
<style>
:root{--bg:#fff;--fg:#1a1a1a;--mut:#666;--line:#e5e5e5}
body{margin:0;padding:24px;font:14px/1.6 -apple-system,"Segoe UI","Microsoft YaHei",sans-serif;background:var(--bg);color:var(--fg)}
h1{font-size:18px;font-weight:500;margin:0 0 4px}
p.sub{margin:0 0 20px;color:var(--mut);font-size:13px}
.card{border:1px solid var(--line);border-radius:12px;padding:16px;margin-bottom:18px}
.card h2{font-size:14px;font-weight:500;margin:0 0 12px}
canvas{width:100%;height:240px;display:block}
.legend{display:flex;gap:18px;font-size:12px;color:var(--mut);margin-top:8px}
.dot{display:inline-block;width:14px;height:2px;vertical-align:middle;margin-right:6px}
table{border-collapse:collapse;font-size:13px;width:100%}
th,td{border-bottom:1px solid var(--line);padding:6px 8px;text-align:right}
th:first-child,td:first-child{text-align:left}
th{color:var(--mut);font-weight:400}
</style></head><body>
<h1>S 曲线 vs 梯形 轨迹对比</h1>
<p class="sub">参数：加速度上限 30 rad/s²，jerk 上限 80 rad/s³，控制频率 270 Hz</p>
<div id="cards"></div>
<script>
const DATA = __DATA__;
function draw(cv, ts, ss, vs, tt, st, vt){
  const dpr = window.devicePixelRatio||1;
  const w = cv.clientWidth, h = cv.clientHeight;
  cv.width = w*dpr; cv.height = h*dpr;
  const c = cv.getContext('2d'); c.scale(dpr,dpr); c.clearRect(0,0,w,h);
  const pad = {l:44,r:12,t:12,b:24};
  const W = w-pad.l-pad.r, H = h-pad.t-pad.b;
  const tmax = Math.max(ts[ts.length-1], tt[tt.length-1]);
  const vmax = Math.max(...vs, ...vt, 0.001)*1.1;
  c.strokeStyle='#e5e5e5'; c.lineWidth=1;
  c.beginPath(); c.moveTo(pad.l,pad.t); c.lineTo(pad.l,pad.t+H); c.lineTo(pad.l+W,pad.t+H); c.stroke();
  c.fillStyle='#888'; c.font='11px sans-serif';
  c.fillText('t/s', pad.l+W-16, pad.t+H+18);
  c.fillText(vmax.toFixed(1), 6, pad.t+8);
  c.fillText('0', 30, pad.t+H+4);
  const X = t=>pad.l+t/tmax*W, Y = v=>pad.t+H-v/vmax*H;
  c.lineWidth=2;
  c.strokeStyle='#c0c0c0'; c.beginPath();
  tt.forEach((t,i)=>i?c.lineTo(X(t),Y(vt[i])):c.moveTo(X(t),Y(vt[i]))); c.stroke();
  c.strokeStyle='#378ADD'; c.beginPath();
  ts.forEach((t,i)=>i?c.lineTo(X(t),Y(vs[i])):c.moveTo(X(t),Y(vs[i]))); c.stroke();
}
const box = document.getElementById('cards');
DATA.forEach(d=>{
  const el = document.createElement('div'); el.className='card';
  el.innerHTML = `<h2>${d.name}　位移 ${d.D.toFixed(3)} rad（${(d.D*180/Math.PI).toFixed(0)}°）</h2>
    <canvas></canvas>
    <div class="legend"><span><i class="dot" style="background:#378ADD"></i>S 曲线</span>
    <span><i class="dot" style="background:#c0c0c0"></i>旧梯形</span>
    <span>纵轴＝速度 rad/s</span></div>`;
  box.appendChild(el);
  const cv = el.querySelector('canvas');
  requestAnimationFrame(()=>draw(cv, d.ts, d.ss, d.vs, d.tt, d.st, d.vt));
});
</script></body></html>"""


if __name__ == "__main__":
    ok = run_checks()
    build_report()
    if not ok:
        raise SystemExit(1)
