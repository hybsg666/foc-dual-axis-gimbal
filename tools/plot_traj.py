# -*- coding: utf-8 -*-
"""读 ESP32 10Hz CSV 遥测，画 yaw-pitch 平面轨迹并算跟随性能。
用法: python tools/plot_traj.py <log> [out.svg] [--x0 a --x1 b]
CSV 列: 0=mode 1=yaw_tgt 2=yaw_act 3=yaw_vel 4=iq 5=iqcmd 6=fault
        7=mode 8=pitch_tgt 9=pitch_act 10=pitch_vel 11=iq 12=iqcmd 13=fault
单位都是弧度。
"""
import math
import sys


def load(path):
    tgt, act, iq = [], [], []
    for line in open(path, encoding='utf-8', errors='replace'):
        line = line.strip()
        if not line or not line[0].isdigit():
            continue
        p = line.split(',')
        if len(p) < 11:
            continue
        try:
            v = [float(x) for x in p[:14]]
        except ValueError:
            continue
        tgt.append((math.degrees(v[1]), math.degrees(v[8])))
        act.append((math.degrees(v[2]), math.degrees(v[9])))
        iq.append(max(abs(v[4]), abs(v[11])))
    return tgt, act, iq


def stats(tgt, act):
    n = len(tgt)
    ey = [act[i][0] - tgt[i][0] for i in range(n)]
    ep = [act[i][1] - tgt[i][1] for i in range(n)]
    ms = lambda e: math.sqrt(sum(x * x for x in e) / len(e))
    ys = [p[0] for p in tgt]
    ps = [p[1] for p in tgt]
    return dict(n=n, secs=n * 0.1,
                yaw=(min(ys), max(ys)), pitch=(min(ps), max(ps)),
                ey_rms=ms(ey), ey_max=max(abs(x) for x in ey),
                ep_rms=ms(ep), ep_max=max(abs(x) for x in ep))


def crossings(vals, level):
    """返回穿过 level 的时刻（采样序号，线性插值）"""
    out = []
    for i in range(1, len(vals)):
        a, b = vals[i - 1] - level, vals[i] - level
        if a * b < 0:
            out.append(i - 1 + abs(a) / (abs(a) + abs(b)))
    return out


def period_ratio(tgt):
    """用 yaw / pitch 目标的过中点次数估周期比（8 字应为 2:1）"""
    ys = [p[0] for p in tgt]
    ps = [p[1] for p in tgt]
    my, mp = (min(ys) + max(ys)) / 2, (min(ps) + max(ps)) / 2
    cy, cp = crossings(ys, my), crossings(ps, mp)
    # 上升沿数 = 完整周期数
    def ups(vals, lv):
        c = 0
        for i in range(1, len(vals)):
            if vals[i - 1] < lv <= vals[i]:
                c += 1
        return c
    return ups(ys, my), ups(ps, mp), len(cy), len(cp)


def radius_series(tgt, cx, cy):
    return [math.hypot(p[0] - cx, p[1] - cy) for p in tgt]


def svg_panel(pts, x0, y0, w, h, lo, hi, color, dash=''):
    def mx(v):
        return x0 + (v - lo) / (hi - lo) * w

    def my(v):
        return y0 + h - (v - lo) / (hi - lo) * h
    d = ' '.join('%.2f,%.2f' % (mx(x), my(y)) for x, y in pts)
    da = ' stroke-dasharray="%s"' % dash if dash else ''
    return '<polyline points="%s" fill="none" stroke="%s" stroke-width="1.6"%s ' \
           'stroke-linejoin="round" stroke-linecap="round"/>' % (d, color, da)


def transform_abs(pts):
    """转过 90°：把 8 字 / 螺旋转成竖着，看形状更直观（不改变几何）"""
    return [(-p[1], p[0]) for p in pts]


if __name__ == '__main__':
    path = sys.argv[1]
    out = sys.argv[2] if len(sys.argv) > 2 else None
    tgt, act, iq = load(path)
    st = stats(tgt, act)
    print('samples %d  (%.1f s)' % (st['n'], st['secs']))
    print('yaw   target %.1f .. %.1f' % st['yaw'])
    print('pitch target %.1f .. %.1f' % st['pitch'])
    print('trace yaw   err rms %.2f  max %.2f deg' % (st['ey_rms'], st['ey_max']))
    print('trace pitch err rms %.2f  max %.2f deg' % (st['ep_rms'], st['ep_max']))
    print('iq max %.2f A' % max(iq))
    uy, up, cy, cp = period_ratio(tgt)
    print('crossings: yaw up %d / pitch up %d  (pitch/yaw = %.2f)'
          % (uy, up, (up / uy) if uy else 0))
    # 半径包络（相对窗口中心）
    cx = (st['yaw'][0] + st['yaw'][1]) / 2
    cy_ = (st['pitch'][0] + st['pitch'][1]) / 2
    r = radius_series(tgt, cx, cy_)
    print('radius from (%.1f,%.1f): start %.1f  max %.1f  end %.1f'
          % (cx, cy_, r[0], max(r), r[-1]))
    # 逐段最大半径（5 段），用于确认"扩到最大再收回"
    seg = max(1, len(r) // 6)
    print('radius profile: ' + '  '.join('%.1f' % max(r[i:i + seg])
                                         for i in range(0, len(r), seg)))

    if out:
        W, H = 720, 400
        pw, ph = 300, 300
        lo, hi = 50.0, 130.0
        s = ['<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 %d %d" '
             'font-family="system-ui,sans-serif">' % (W, H),
             '<rect width="%d" height="%d" fill="#ffffff"/>' % (W, H)]
        for k, (title, pan_x) in enumerate((('figure-8  (key i)', 40),
                                            ('spiral  (key w)', 380))):
            x0, y0 = pan_x, 60
            s.append('<rect x="%d" y="%d" width="%d" height="%d" fill="#fafafa" '
                     'stroke="#e5e5e5"/>' % (x0, y0, pw, ph))
            for g in range(60, 131, 10):
                gx = x0 + (g - lo) / (hi - lo) * pw
                gy = y0 + ph - (g - lo) / (hi - lo) * ph
                s.append('<line x1="%.1f" y1="%d" x2="%.1f" y2="%d" stroke="#ececec"/>'
                         % (gx, y0, gx, y0 + ph))
                s.append('<line x1="%d" y1="%.1f" x2="%d" y2="%.1f" stroke="#ececec"/>'
                         % (x0, gy, x0 + pw, gy))
            s.append('<text x="%d" y="%d" font-size="13" fill="#333">%s</text>'
                     % (x0, y0 - 12, title))
            pts = tgt if k == 0 else tgt
            if k == 1:
                pts = tgt
            s.append(svg_panel(tgt, x0, y0, pw, ph, lo, hi, '#b0b0b0', '4 3'))
            s.append(svg_panel(act, x0, y0, pw, ph, lo, hi, '#d93b3b'))
            s.append('<text x="%d" y="%d" font-size="11" fill="#888">yaw -&gt;</text>'
                     % (x0 + pw - 44, y0 + ph + 16))
            s.append('<text x="%d" y="%d" font-size="11" fill="#888">pitch</text>'
                     % (x0 - 30, y0 + 10))
        s.append('<text x="40" y="30" font-size="13" fill="#444">'
                 'grey dashed = target, red = actual</text>')
        s.append('</svg>')
        open(out, 'w', encoding='utf-8').write('\n'.join(s))
        print('svg ->', out)
