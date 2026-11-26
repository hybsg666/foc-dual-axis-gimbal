#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FOC 上位机 —— 量化测试工具（第十步）

功能：
  1. 采数据：读串口遥测（5 列 CSV），带主机时间戳，存 CSV
  2. 发指令：按脚本发 v/p/i 指令，做阶跃测试
  3. 算指标：上升时间、超调、调节时间、稳态误差、稳态纹波
  4. 出图：目标 vs 实际曲线，存 PNG

用法（在 D:/FOC/MyFOC 目录下执行）：

    # 位置环阶跃（p0 -> 1.57 -> -1.57 -> 0）
    python tools/host/foc_tool.py -p COM3 --test pos

    # 速度环阶跃（v0 -> v10 -> v-10）
    python tools/host/foc_tool.py -p COM3 --test vel

    # 位置保持（测稳态误差/纹波；期间可以用手拨一下测抗扰）
    python tools/host/foc_tool.py -p COM3 --test hold -d 10

    # 只发一条命令，观察几秒
    python tools/host/foc_tool.py -p COM3 --cmd v10 --watch 5

    # 只看曲线不弹窗（默认就是只存 PNG）
    python tools/host/foc_tool.py -p COM3 --test pos --show

输出目录：runs/<时间戳>_<测试名>/
    data.csv      原始数据（t, 目标, 反馈, 速度, iq, 故障码）
    plot.png      曲线
    summary.txt   指标汇总（简历上的数字就从这里抄）

注意事项：
  - 运行前必须先关掉 PlatformIO / VSCode 的串口监视器，否则端口被占用。
  - ESP32-S3 是原生 USB CDC，波特率填多少都行。
"""

import argparse
import csv
import os
import statistics
import sys
import time
from datetime import datetime

# pyserial 只在真正连串口时才需要（这样纯数据分析部分可以脱离串口独立跑/单测）
try:
    import serial
except ImportError:
    serial = None


# ----------------------------------------------------------------------
# 串口 + 采集
# ----------------------------------------------------------------------
class Scope:
    """薄封装：发命令 + 按时间窗采数据"""

    def __init__(self, port, baud=115200):
        if serial is None:
            print("缺少 pyserial，请先安装：pip install pyserial")
            sys.exit(1)
        try:
            self.ser = serial.Serial(port, baud, timeout=0.05)
        except serial.SerialException as e:
            print(f"\n[错误] 打不开串口 {port}：{e}")
            print("       最常见原因：串口监视器还开着。请先关掉，再运行本脚本。\n")
            sys.exit(2)
        time.sleep(0.4)
        self.ser.reset_input_buffer()
        self.t0 = time.monotonic()

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass

    @staticmethod
    def parse(line):
        """'0.0,0.0,0.01,0.0,0' -> [0.0, 0.0, 0.01, 0.0, 0.0]；非数据行返回 None"""
        parts = line.split(",")
        if len(parts) != 5:
            return None
        try:
            return [float(x) for x in parts]
        except ValueError:
            return None

    def send(self, cmd):
        self.ser.write((cmd + "\n").encode())
        self.ser.flush()

    def pump(self, duration):
        """采 duration 秒，返回 [(t, [5 列])]"""
        rows = []
        end = time.monotonic() + duration
        while time.monotonic() < end:
            raw = self.ser.readline()
            if not raw:
                continue
            s = raw.decode("utf-8", "ignore").strip()
            if not s:
                continue
            vals = self.parse(s)
            if vals is None:
                continue
            rows.append((time.monotonic() - self.t0, vals))
        return rows


# ----------------------------------------------------------------------
# 指标计算
# ----------------------------------------------------------------------
def step_metrics(rows, target, col=1):
    """
    对一段阶跃响应算指标。rows=[(t, vals)]，col 是反馈列下标（0=目标,1=反馈）。
    返回 dict，数据不足返回 None。
    """
    if len(rows) < 10:
        return None
    t = [r[0] for r in rows]
    y = [r[1][col] for r in rows]

    # 起点取最开始几个点的中位数（点越多越稳，但也越容易把"已经起跳"的点算进起点）
    n0 = min(4, max(2, len(y) // 50))
    y_start = statistics.median(y[:n0])
    step = target - y_start
    if abs(step) < 1e-6:
        return None

    def prog(v):            # 归一化进度：0=起点，1=目标（自动适配正负阶跃）
        return (v - y_start) / step

    p = [prog(v) for v in y]
    i10 = next((i for i, v in enumerate(p) if v >= 0.10), None)
    i90 = next((i for i, v in enumerate(p) if v >= 0.90), None)
    rise = (t[i90] - t[i10]) if (i10 is not None and i90 is not None) else float("nan")

    peak = max(p)
    overshoot = max(0.0, (peak - 1.0) * 100.0)

    # 调节时间：最后一次偏离 ±2% 带的时刻距段开始
    tol = 0.02
    i_last_out = 0
    for i, v in enumerate(p):
        if abs(v - 1.0) > tol:
            i_last_out = i
    settle = t[i_last_out] - t[0]

    tail = y[-max(5, len(y) // 5):]
    y_ss = sum(tail) / len(tail)
    ss_err = y_ss - target
    ripple = statistics.pstdev(tail) if len(tail) > 1 else 0.0

    fault = max(abs(r[1][4]) for r in rows)

    return dict(
        n=len(y), dur=t[-1] - t[0],
        target=target, y_start=y_start, y_ss=y_ss,
        rise=rise, overshoot=overshoot, settle=settle,
        ss_err=ss_err, ripple=ripple, fault=fault,
    )


def hold_metrics(rows, target, col=1):
    """
    保持类测试（如 p0 静止保持）：不看阶跃，只看后半段的稳态表现。
    返回稳态均值误差、纹波、最大偏差。
    """
    if len(rows) < 10:
        return None
    tail = [r[1][col] for r in rows][-max(5, len(rows) // 2):]
    y_ss = sum(tail) / len(tail)
    ripple = statistics.pstdev(tail) if len(tail) > 1 else 0.0
    peak_dev = max(abs(v - target) for v in tail)
    fault = max(abs(r[1][4]) for r in rows)
    return dict(n=len(rows), y_ss=y_ss, target=target, err=y_ss - target,
                ripple=ripple, peak_dev=peak_dev, fault=fault)


def fmt_hold(title, m):
    return "\n".join([
        f"--- {title} ---",
        f"  样本数         {m['n']} 点",
        f"  目标           {m['target']:.4f}",
        f"  稳态均值       {m['y_ss']:.4f}",
        f"  稳态误差       {m['err']:+.4f}",
        f"  稳态纹波(σ)    {m['ripple']:.4f}",
        f"  最大偏差       {m['peak_dev']:.4f}",
        f"  故障码峰值     {m['fault']:.0f}" + ("   <== 有故障！" if m['fault'] else ""),
    ])


def fmt_metrics(title, m):
    lines = [
        f"--- {title} ---",
        f"  样本/时长      {m['n']} 点 / {m['dur']:.2f} s",
        f"  起点 -> 目标   {m['y_start']:.4f} -> {m['target']:.4f}",
        f"  上升时间       {m['rise'] * 1000:.0f} ms   (10%~90%)",
        f"  超调           {m['overshoot']:.1f} %",
        f"  调节时间       {m['settle'] * 1000:.0f} ms   (±2%)",
        f"  稳态误差       {m['ss_err']:+.4f}",
        f"  稳态纹波(σ)    {m['ripple']:.4f}",
        f"  故障码峰值     {m['fault']:.0f}" + ("   <== 有故障！" if m['fault'] else ""),
    ]
    return "\n".join(lines)


# ----------------------------------------------------------------------
# 测试脚本定义：(持续时间 s, 要发的命令, 分析规格)
#   分析规格 = (反馈列下标, 目标值) 或 None
# ----------------------------------------------------------------------
TESTS = {
    "pos": [
        (0.6, "v0", None),          # 先确保速度模式目标为 0
        (2.5, "p0", None),          # 回零并稳定
        (2.5, "p1.57", (1, 1.57)),  # +90°，看阶跃
        (2.5, "p-1.57", (1, -1.57)),  # -90°
        (2.5, "p0", (1, 0.0)),      # 回零
    ],
    "vel": [
        (0.6, "v0", None),
        (2.0, "v10", (1, 10.0)),
        (2.0, "v0", (1, 0.0)),
        (2.0, "v-10", (1, -10.0)),
        (2.0, "v0", (1, 0.0)),
    ],
}


# ----------------------------------------------------------------------
# 出图
# ----------------------------------------------------------------------
def make_html_chart(segs, path):
    """零依赖出图：纯 Python 生成内联 SVG 的 HTML，双击即可看（不需要 matplotlib）"""
    W, H, PAD = 900, 240, 44
    parts = ["<!DOCTYPE html><html><head><meta charset='utf-8'>",
             "<title>FOC 曲线</title>",
             "<style>body{background:#1b1b1f;color:#ddd;font-family:Segoe UI,sans-serif;"
             "margin:24px}h3{font-weight:500;font-size:14px;color:#8ab4f8}"
             "svg{background:#232329;border-radius:6px}text{fill:#999;font-size:11px}"
             "</style></head><body><h2>FOC 量化测试曲线</h2>"]

    for title, rows in segs:
        if len(rows) < 2:
            continue
        t = [r[0] - rows[0][0] for r in rows]
        ys = [r[1][1] for r in rows]
        yt = [r[1][0] for r in rows]
        tmax = max(t) or 1.0
        vals = ys + yt
        vmin, vmax = min(vals), max(vals)
        if abs(vmax - vmin) < 1e-9:
            vmax = vmin + 1.0
        span = vmax - vmin
        vmin -= span * 0.05
        vmax += span * 0.05

        def px(tv):
            return PAD + (tv / tmax) * (W - 2 * PAD)

        def py(v):
            return H - PAD - ((v - vmin) / (vmax - vmin)) * (H - 2 * PAD)

        def poly(arr, color, dash, width):
            pts = " ".join(f"{px(x):.1f},{py(y):.1f}" for x, y in zip(t, arr))
            d = f" stroke-dasharray='6,4'" if dash else ""
            return (f"<polyline points='{pts}' fill='none' stroke='{color}' "
                    f"stroke-width='{width}'{d}/>")

        parts.append(f"<h3>{title}</h3>")
        parts.append(f"<svg width='{W}' height='{H}'>")
        # 边框与刻度
        parts.append(f"<rect x='{PAD}' y='{PAD}' width='{W-2*PAD}' height='{H-2*PAD}' "
                     f"fill='none' stroke='#3a3a44'/>")
        parts.append(f"<text x='6' y='{py(vmax)+4:.0f}'>{vmax:.2f}</text>")
        parts.append(f"<text x='6' y='{py(vmin)+4:.0f}'>{vmin:.2f}</text>")
        parts.append(f"<text x='{W//2}' y='{H-8}'>t = {tmax:.2f} s</text>")
        parts.append(poly(yt, "#f5a623", True, 1.4))    # 目标：橙色虚线
        parts.append(poly(ys, "#4fc3f7", False, 2.0))   # 实际：蓝色实线
        parts.append("</svg>")

    parts.append("<p style='color:#777;font-size:12px'>橙色虚线 = 目标，蓝色实线 = 实际</p>")
    parts.append("</body></html>")
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(parts))
    print(f"  曲线: {path}  (双击用浏览器打开)")


def make_plot(segs, path, show=False):
    try:
        import matplotlib
        matplotlib.use("TkAgg" if show else "Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        # 没装 matplotlib 就退化成零依赖的 HTML 曲线，保证一定能看到图
        make_html_chart(segs, os.path.splitext(path)[0] + ".html")
        return

    n = len(segs)
    if n == 0:
        return
    fig, axes = plt.subplots(n, 1, figsize=(9, 2.6 * n), squeeze=False)
    for ax, (title, rows) in zip(axes[:, 0], segs):
        t = [r[0] - rows[0][0] for r in rows]
        ax.plot(t, [r[1][0] for r in rows], "--", lw=1.2, label="target")
        ax.plot(t, [r[1][1] for r in rows], "-", lw=1.6, label="actual")
        ax.set_title(title, fontsize=9)
        ax.set_xlabel("t (s)", fontsize=8)
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    print(f"  曲线: {path}")
    if show:
        plt.show()
    plt.close(fig)


# ----------------------------------------------------------------------
# 主流程
# ----------------------------------------------------------------------
def load_csv_any(path, fs=200.0, time_col=None):
    """
    读任意 CSV（VOFA+ 导出 / 本工具的 data.csv 都行），返回 rows=[(t, [v1..v5])]。

    没有时间列时用 --fs 采样率合成时间轴；列数不足 5 列自动补 0。
    """
    import re
    rows, t_acc = [], 0.0
    with open(path, "r", encoding="utf-8-sig", errors="ignore") as f:
        for line in f:
            s = line.strip()
            if not s or s[0] in "#/":
                continue
            parts = [x for x in re.split(r"[,\t; ]+", s) if x]
            vals = []
            for x in parts:
                try:
                    vals.append(float(x))
                except ValueError:
                    vals = []          # 只要有一个非数字，整行当表头丢掉
                    break
            if not vals:
                continue
            if time_col is not None and time_col < len(vals):
                t = vals[time_col]
                vals = vals[:time_col] + vals[time_col + 1:]
            else:
                t = t_acc
                t_acc += 1.0 / fs
            vals = (vals + [0.0] * 5)[:5]
            rows.append((t, vals))
    # 时间轴归一化到从 0 开始
    if rows:
        t0 = rows[0][0]
        rows = [(t - t0, v) for t, v in rows]
    return rows


def main():
    ap = argparse.ArgumentParser(description="FOC 上位机：量化测试")
    ap.add_argument("-p", "--port", default="COM3", help="串口，如 COM3")
    ap.add_argument("--from-csv", help="不从串口采，直接分析一个 CSV（VOFA+ 导出的也行）")
    ap.add_argument("--fs", type=float, default=200.0,
                    help="CSV 没有时间列时的采样率 Hz（默认 200，即 5ms 一点）")
    ap.add_argument("--time-col", type=int, default=None,
                    help="CSV 里时间列的列号（0 起），不给就按 --fs 合成")
    ap.add_argument("--col", type=int, default=1, help="反馈量所在列（0 起，默认 1）")
    ap.add_argument("--target", type=float, help="阶跃目标值（配合 --from-csv）")
    ap.add_argument("--unit", default="", help="单位，仅用于打印")
    ap.add_argument("-b", "--baud", type=int, default=115200)
    ap.add_argument("--test", choices=sorted(TESTS.keys()) + ["hold"],
                    help="跑预设测试：pos / vel / hold")
    ap.add_argument("-d", "--duration", type=float, default=8.0,
                    help="hold 测试的持续时间（秒）")
    ap.add_argument("--cmd", help="只发这一条命令（如 v10 / p1.57 / i0.3）")
    ap.add_argument("--watch", type=float, default=3.0, help="配合 --cmd，采多少秒")
    ap.add_argument("--show", action="store_true", help="弹窗显示曲线")
    ap.add_argument("--outdir", default="runs", help="输出根目录")
    args = ap.parse_args()

    # ---------------- 直接分析已有 CSV（VOFA+ 导出）----------------
    if args.from_csv:
        rows = load_csv_any(args.from_csv, args.fs, args.time_col)
        if len(rows) < 10:
            print("  这个 CSV 里没读到足够的数据行")
            return
        dt = (rows[-1][0] - rows[0][0]) / max(len(rows) - 1, 1)
        print(f"[分析] {args.from_csv}")
        print(f"  共 {len(rows)} 点，采样间隔 {dt * 1000:.1f} ms（约 {1 / dt:.0f} Hz）")
        if args.target is None:
            # 不给 target 就当成保持测试：以尾部均值为目标
            m = hold_metrics(rows, rows[-1][1][args.col], args.col)
            print(fmt_hold(f"hold / col{args.col}", m) if m else "  数据不足")
        else:
            m = step_metrics(rows, args.target, args.col)
            if not m:
                print("  数据不足，或这不像一次阶跃")
                return
            print(fmt_metrics(f"step / col{args.col} (target {args.target:g} {args.unit})", m))
            print(f"\n  ⚠ 采样间隔 {dt * 1000:.1f} ms 是本表所有时间的分辨率下限："
                  f"任何小于 {dt * 1000 * 2:.0f} ms 的指标都只有 1~2 个点，不可信。")
        return

    if not args.test and not args.cmd:
        ap.print_help()
        return

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    tag = args.test or "manual"
    outdir = os.path.join(args.outdir, f"{stamp}_{tag}")
    os.makedirs(outdir, exist_ok=True)

    sc = Scope(args.port, args.baud)
    print(f"[连接] {args.port} @ {args.baud}   输出目录: {outdir}")

    # ---------------- 只发一条命令 ----------------
    if args.cmd and not args.test:
        print(f"  >> {args.cmd}")
        sc.send(args.cmd)
        rows = sc.pump(args.watch)
        sc.close()
        write_csv(os.path.join(outdir, "data.csv"), [("cmd:" + args.cmd, rows)])
        if rows:
            print(f"  采到 {len(rows)} 点，约 {len(rows) / max(rows[-1][0] - rows[0][0], 1e-6):.0f} Hz")
        make_plot([(f"cmd {args.cmd}", rows)], os.path.join(outdir, "plot.png"), args.show)
        return

    # ---------------- 跑预设测试 ----------------
    if args.test == "hold":
        seq = [(0.6, "v0", None), (args.duration, "p0", (1, 0.0))]
        print("\n  提示：采集中可以用手轻拨轴，观察抗扰与回位（不影响稳态误差统计）\n")
    else:
        seq = TESTS[args.test]

    segs = []
    for dur, cmd, spec in seq:
        if cmd:
            print(f"  >> {cmd:8s} 采集 {dur:.1f}s ...")
            sc.send(cmd)
        rows = sc.pump(dur)
        segs.append((cmd or "(start)", spec, rows))
    sc.close()

    # 写 CSV
    write_csv(os.path.join(outdir, "data.csv"), [(c, r) for c, _, r in segs])

    # 算指标
    report, plot_segs = [], []
    unit = "rad/s" if args.test == "vel" else "rad"
    is_hold = (args.test == "hold")
    for cmd, spec, rows in segs:
        if not spec or not rows:
            continue
        col, target = spec
        if is_hold:
            m = hold_metrics(rows, target, col)
            if m:
                report.append(fmt_hold(f"hold / p0  (target {target:g} {unit})", m))
        else:
            m = step_metrics(rows, target, col)
            if m:
                title = f"{args.test} / {cmd}  (target {target:g} {unit})"
                report.append(fmt_metrics(title, m))
                plot_segs.append((title, rows))

    # 打印 + 存
    summary = []
    if report:
        summary.append(f"FOC 量化测试  {stamp}  test={args.test}  port={args.port}")
        summary.append("")
        summary.extend(report)
        text = "\n".join(summary)
        print("\n" + "=" * 62)
        print(text)
        print("=" * 62)
        with open(os.path.join(outdir, "summary.txt"), "w", encoding="utf-8") as f:
            f.write(text + "\n")
        print(f"  指标: {os.path.join(outdir, 'summary.txt')}")
    else:
        print("  本段没有可分析的阶跃数据")

    make_plot(plot_segs, os.path.join(outdir, "plot.png"), args.show)


def write_csv(path, segs):
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "segment", "col1", "col2", "col3", "col4", "col5"])
        for i, (name, rows) in enumerate(segs):
            for t, v in rows:
                w.writerow([f"{t:.5f}", i] + [f"{x:.5f}" for x in v])
    print(f"  数据: {path}")


if __name__ == "__main__":
    main()
