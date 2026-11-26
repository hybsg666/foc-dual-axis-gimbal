# -*- coding: utf-8 -*-
"""
把 ESP32 串口日志里的 CSV 遥测行解析出来，画 yaw-pitch 平面轨迹，判断画的是不是圆。

用法:
    python tools/plot_circle.py <日志文件> [输出.svg]

日志里认两种行:
  1) CSV 遥测（每 100ms 一行，推荐）
     2,1.5708,1.5690,0.01,0.02,0.03,0, 2,-1.5708,-1.5700,0.00,0.01,0.02,0
     字段: mode,target,pos,vel,iq,iqcmd,fault  ×2（弧度）
  2) [STAT] 文本（按 `?` 才有）
     [STAT] m0 mode=POSITION ... target=90.0deg now=89.9deg ...

输出:
  - 控制台: 中心、半径、圆度、最大偏差、采样点数
  - SVG: 目标轨迹 vs 实际轨迹（同一张图）
"""
import sys, re, math, os

def parse(path):
    tgt = []   # [(yaw_deg, pitch_deg)]
    act = []
    stat_t = {}
    txt = open(path, 'rb').read().decode('utf-8', 'replace')

    for line in txt.splitlines():
        line = line.strip()
        # ---- CSV ----
        if re.match(r'^\d+,', line):
            parts = line.split(',')
            if len(parts) >= 13:
                try:
                    f = [float(p) for p in parts[:13]]
                except ValueError:
                    continue
                # 0:mode 1:target0 2:pos0 3:vel0 4:iq0 5:iqcmd0 6:fault0
                # 7:mode 8:target1 9:pos1 ...
                tgt.append((math.degrees(f[1]), math.degrees(f[8])))
                act.append((math.degrees(f[2]), math.degrees(f[9])))
            continue
        # ---- [STAT] ----
        m = re.search(r'\[STAT\] m(\d) .*?target=(-?[\d.]+)deg now=(-?[\d.]+)deg', line)
        if m:
            i = int(m.group(1))
            stat_t.setdefault(i, []).append((float(m.group(2)), float(m.group(3))))
    return tgt, act, stat_t


def fit_circle(pts):
    """Kasa 代数圆拟合：x²+y² = 2ax + 2by + c  → 最小二乘解 (a,b,c)。
    比"取均值当圆心"稳得多：只跑到 1.5 圈、圆弧不闭合时均值会偏。"""
    n = len(pts)
    sx = sy = sxx = syy = sxy = sxz = syz = sz = 0.0
    for x, y in pts:
        z = x * x + y * y
        sx += x;  sy += y;  sz += z
        sxx += x * x;  syy += y * y;  sxy += x * y
        sxz += x * z;  syz += y * z
    # 正规方程
    A = [[sxx, sxy, sx],
         [sxy, syy, sy],
         [sx,  sy,  float(n)]]
    B = [sxz, syz, sz]
    # 3x3 高斯消元
    for i in range(3):
        p = max(range(i, 3), key=lambda r: abs(A[r][i]))
        if abs(A[p][i]) < 1e-12:
            return None
        A[i], A[p] = A[p], A[i]
        B[i], B[p] = B[p], B[i]
        for r in range(i + 1, 3):
            f = A[r][i] / A[i][i]
            for c in range(i, 3):
                A[r][c] -= f * A[i][c]
            B[r] -= f * B[i]
    sol = [0.0, 0.0, 0.0]
    for i in (2, 1, 0):
        sol[i] = (B[i] - sum(A[i][j] * sol[j] for j in range(i + 1, 3))) / A[i][i]
    # z = 2a·x + 2b·y + c，所以解出来的前两个系数要除以 2
    a, b, c = sol[0] * 0.5, sol[1] * 0.5, sol[2]
    r2 = c + a * a + b * b
    if r2 <= 0:
        return None
    return a, b, math.sqrt(r2)


def circle_stats(pts):
    n = len(pts)
    if n < 8:
        return None
    fit = fit_circle(pts)
    if fit:
        cx, cy, rm = fit
    else:
        cx = sum(p[0] for p in pts) / n
        cy = sum(p[1] for p in pts) / n
        rm = sum(math.hypot(p[0] - cx, p[1] - cy) for p in pts) / n
    r = [math.hypot(p[0] - cx, p[1] - cy) for p in pts]
    dev = max(abs(x - rm) for x in r)
    # 覆盖了多少圈：把 360° 分成 12 个扇区，看走过几个（只在时间顺序正确时有意义）
    sectors = set()
    for x, y in pts:
        a = math.atan2(y - cy, x - cx)
        sectors.add(int((a + math.pi) / (2 * math.pi) * 12) % 12)
    return dict(n=n, cx=cx, cy=cy, r=rm, rmin=min(r), rmax=max(r), dev=dev,
                dev_pct=(dev / rm * 100.0) if rm else 0.0, cover=len(sectors) / 12.0)


def svg(tgt, act, out):
    W, H, PAD = 620, 620, 60
    allp = tgt + act
    if not allp:
        print('no points'); return
    xs = [p[0] for p in allp]; ys = [p[1] for p in allp]
    lo_x, hi_x = min(xs) - 8, max(xs) + 8
    lo_y, hi_y = min(ys) - 8, max(ys) + 8
    span = max(hi_x - lo_x, hi_y - lo_y)
    cxs, cys = (lo_x + hi_x) / 2, (lo_y + hi_y) / 2
    lo_x, hi_x = cxs - span / 2, cxs + span / 2
    lo_y, hi_y = cys - span / 2, cys + span / 2

    def X(v): return PAD + (v - lo_x) / (hi_x - lo_x) * (W - 2 * PAD)
    def Y(v): return H - PAD - (v - lo_y) / (hi_y - lo_y) * (H - 2 * PAD)

    def path(points, close=False):
        if not points: return ''
        d = 'M %.2f %.2f ' % (X(points[0][0]), Y(points[0][1]))
        d += ' '.join('L %.2f %.2f' % (X(p[0]), Y(p[1])) for p in points[1:])
        if close: d += ' Z'
        return d

    s = ['<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 %d %d" width="%d" height="%d">' % (W, H, W, H)]
    s.append('<rect width="100%" height="100%" fill="#ffffff"/>')
    # 网格
    for k in range(5):
        gx = lo_x + (hi_x - lo_x) * k / 4
        gy = lo_y + (hi_y - lo_y) * k / 4
        s.append('<line x1="%.1f" y1="%d" x2="%.1f" y2="%d" stroke="#eeeeee"/>' % (X(gx), PAD, X(gx), H - PAD))
        s.append('<line x1="%d" y1="%.1f" x2="%d" y2="%.1f" stroke="#eeeeee"/>' % (PAD, Y(gy), W - PAD, Y(gy)))
        s.append('<text x="%.1f" y="%d" font-size="11" fill="#999" text-anchor="middle">%.0f°</text>' % (X(gx), H - PAD + 18, gx))
        s.append('<text x="%d" y="%.1f" font-size="11" fill="#999" text-anchor="end">%.0f°</text>' % (PAD - 6, Y(gy) + 4, gy))
    s.append('<rect x="%d" y="%d" width="%d" height="%d" fill="none" stroke="#cccccc"/>' % (PAD, PAD, W - 2 * PAD, H - 2 * PAD))
    # 目标（灰虚线）
    s.append('<path d="%s" fill="none" stroke="#9e9e9e" stroke-width="1.6" stroke-dasharray="5 4"/>' % path(tgt, True))
    # 实际（红实线）
    s.append('<path d="%s" fill="none" stroke="#e53935" stroke-width="2.2"/>' % path(act))
    for p in act[::max(1, len(act) // 60)]:
        s.append('<circle cx="%.2f" cy="%.2f" r="1.8" fill="#e53935"/>' % (X(p[0]), Y(p[1])))
    s.append('<text x="%d" y="26" font-size="15" fill="#222">Yaw–Pitch 平面轨迹（灰虚线=目标，红实线=实际）</text>' %
             (PAD))
    s.append('<text x="%d" y="%d" font-size="12" fill="#666">横轴 yaw(°)，纵轴 pitch(°)；正圆 = 半径恒定</text>' % (PAD, H - 12))
    s.append('</svg>')
    open(out, 'w', encoding='utf-8').write('\n'.join(s))


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(__doc__); sys.exit(1)
    tgt, act, stat = parse(sys.argv[1])
    if not tgt and stat:
        # 退化到 [STAT]：两轴的 target/now 分别按顺序对齐（不是同一时刻，只能粗看）
        n = min(len(stat.get(0, [])), len(stat.get(1, [])))
        tgt = [(stat[0][i][0], stat[1][i][0]) for i in range(n)]
        act = [(stat[0][i][1], stat[1][i][1]) for i in range(n)]
    if not tgt and not act:
        print('!! 日志里没找到遥测行。需要形如  2,1.5708,1.5690,...  的行（每 100ms 一行）')
        print('   检查：串口显示是不是 HEX 模式；是不是把打印关掉了（d 键）')
        txt = open(sys.argv[1], 'rb').read().decode('utf-8', 'replace').splitlines()
        cand = [l for l in txt if ',' in l][:5]
        print('   含逗号的行共 %d 条，前 5 条:' % sum(1 for l in txt if ',' in l))
        for l in cand:
            print('     ', l[:120])
        sys.exit(2)
    print('samples: %d (target) / %d (actual)' % (len(tgt), len(act)))
    for name, pts in (('target', tgt), ('actual', act)):
        st = circle_stats(pts)
        if st:
            print('%-7s n=%d  center=(%.1f, %.1f)  r=%.1f°  [%.1f~%.1f]  max_dev=%.1f° (%.0f%%)  cover=%.0f%%'
                  % (name, st['n'], st['cx'], st['cy'], st['r'], st['rmin'], st['rmax'],
                     st['dev'], st['dev_pct'], st['cover'] * 100))
    out = sys.argv[2] if len(sys.argv) > 2 else 'circle_check.svg'
    svg(tgt, act, out)
    print('wrote', os.path.abspath(out))
