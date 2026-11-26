# -*- coding: utf-8 -*-
"""用 mapbox_earcut 验证带洞三角剖分（面积守恒）"""
import math
import numpy as np
import mapbox_earcut


def _circle_ccw(n, r, cx=0.0, cy=0.0):
    return [(cx + r*math.cos(2*math.pi*i/n), cy + r*math.sin(2*math.pi*i/n)) for i in range(n)]


def _area_of_pts(pts, idx):
    a = 0.0
    for k in range(0, len(idx), 3):
        p1 = pts[idx[k]]; p2 = pts[idx[k+1]]; p3 = pts[idx[k+2]]
        a += 0.5 * ((p2[0]-p1[0])*(p3[1]-p1[1]) - (p3[0]-p1[0])*(p2[1]-p1[1]))
    return abs(a)


def triangulate(outer, holes):
    """outer 逆时针，holes 逆时针。返回 [(p1,p2,p3),...]"""
    pts = outer + [p for h in holes for p in h]
    verts = np.array(pts, dtype=np.float32)
    ring_end = []
    off = len(outer)
    ring_end.append(off)
    for h in holes:
        off += len(h)
        ring_end.append(off)
    ring_end_arr = np.array(ring_end, dtype=np.uint32)
    idx = mapbox_earcut.triangulate_float32(verts, ring_end_arr)
    return [(pts[idx[k]], pts[idx[k+1]], pts[idx[k+2]]) for k in range(0, len(idx), 3)]


if __name__ == '__main__':
    def area_tris(tris):
        a=0.0
        for p1,p2,p3 in tris:
            a += 0.5*((p2[0]-p1[0])*(p3[1]-p1[1])-(p3[0]-p1[0])*(p2[1]-p1[1]))
        return abs(a)

    # 测试3：方板 + 中心孔 r=4
    sq = [(-10,-10),(10,-10),(10,10),(-10,10)]
    tris3 = triangulate(sq, [_circle_ccw(48, 4.0)])
    print(f"测试3 方板带孔: 三角形={len(tris3)} 面积={area_tris(tris3):.3f} 期望={400-math.pi*16:.3f} {'OK' if abs(area_tris(tris3)-(400-math.pi*16))<0.3 else 'FAIL'}")

    # 测试1：圆盘 R=20 + 中央 r=4.7 + 3×M3
    outer = _circle_ccw(96, 20.0)
    holes = [_circle_ccw(48, 4.7)]
    _scw = 10.0/math.sqrt(3.0)
    for hx, hy in [(_scw, 0), (-_scw*0.5, 5), (-_scw*0.5, -5)]:
        holes.append(_circle_ccw(48, 1.7, hx, hy))
    tris = triangulate(outer, holes)
    expect = math.pi*(20**2 - 4.7**2 - 3*1.7**2)
    print(f"测试1 arm水平板: 三角形={len(tris)} 面积={area_tris(tris):.3f} 期望={expect:.3f} {'OK' if abs(area_tris(tris)-expect)<0.5 else 'FAIL'}")

    # 测试2：圆盘 R=21 + 中央 r=14 + 4×M3
    outer2 = _circle_ccw(96, 21.0)
    holes2 = [_circle_ccw(48, 14.0)]
    for sx in (-10.5, 10.5):
        for sy in (-10.5, 10.5):
            holes2.append(_circle_ccw(48, 1.7, sx, sy))
    tris2 = triangulate(outer2, holes2)
    expect2 = math.pi*(21**2 - 14**2 - 4*1.7**2)
    print(f"测试2 arm电机B座: 三角形={len(tris2)} 面积={area_tris(tris2):.3f} 期望={expect2:.3f} {'OK' if abs(area_tris(tris2)-expect2)<0.5 else 'FAIL'}")

    # 测试4：底座带缺口外轮廓 + 中央孔 r=14 + 4×M3
    import math as m
    a0 = m.radians(68); a1 = m.radians(112)
    out_gap = []
    # 外圆大弧 112° -> 68°(绕大圈)
    for i in range(100):
        a = a1 + (a0 + 2*m.pi - a1) * i / 99
        out_gap.append((21*m.cos(a), 21*m.sin(a)))
    # 径向向内 68° -> r=14
    out_gap.append((14*m.cos(a0), 14*m.sin(a0)))
    # 缺口底边 r=14, 68° -> 112°
    for i in range(20):
        a = a0 + (a1 - a0) * i / 19
        out_gap.append((14*m.cos(a), 14*m.sin(a)))
    # 径向向外 112° -> r=21
    out_gap.append((21*m.cos(a1), 21*m.sin(a1)))
    # 中央孔（去掉缺口段）：r=14 的 [112°, 68°] 大弧
    center_hole = []
    for i in range(100):
        a = a1 - (a1 - a0 + 2*m.pi) * i / 99   # 从 112° 顺时针绕大圈到 68°-360°
        center_hole.append((14*m.cos(a), 14*m.sin(a)))
    holes3 = [center_hole]
    for sx in (-10.5, 10.5):
        for sy in (-10.5, 10.5):
            holes3.append(_circle_ccw(48, 1.7, sx, sy))
    tris3 = triangulate(out_gap, holes3)
    # 期望面积 = 圆 R21 - 缺口扇区(外圆 R21 的 [68°,112°] 与 r14 之间) - 中央孔(剩余) - 4 M3
    # 缺口扇区面积 = (112-68)/360 * pi*(21²-14²)
    gap_frac = (112-68)/360.0
    expect3 = math.pi*(21**2 - 14**2) - gap_frac*math.pi*(21**2-14**2) - 4*math.pi*1.7**2
    print(f"测试4 底座带缺口: 三角形={len(tris3)} 面积={area_tris(tris3):.3f} 期望={expect3:.3f} {'OK' if abs(area_tris(tris3)-expect3)<1.0 else 'FAIL'}")
