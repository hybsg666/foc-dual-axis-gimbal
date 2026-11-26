# -*- coding: utf-8 -*-
"""
=====================================================================
 DengFOC 2208 双轴指向云台 —— L 型连接臂 + 激光笔夹座 STL 生成器
=====================================================================
 生成两个零件：
   arm.stl    L 型连接臂
              · 水平盘 套电机A转子顶部（φ9凸台 + 3×M3品字形）  —— A = 偏航(左右)
              · 竖直盘 固定电机B定子（φ28编码器板 + 4×M3）     —— B = 俯仰(上下)
   clamp.stl  激光笔夹座 —— 套电机B【转子顶部】(φ9.6 + 3×M3品字形)，
              托板 + 扎带槽夹激光笔，笔随 B 转子俯仰转动（上下）

 运动关系：
   · 电机A(偏航) 定子固定不动 → A转子带动 L臂 + B电机 + 笔 一起左右转
   · 电机B(俯仰) 定子被 L臂竖直盘固定 → B转子带动笔 上下转

 电机真实结构（DengFOC 2208，空心轴盘式外转子云台电机）：
   · 底部(定子,固定端) = φ30 圆形 AS5600 编码器 PCB + 白色6针插座 + 4×M3铜螺柱
                         (正方形,相邻21mm/对角30mm) + 中央固定螺丝头
   · 顶部(转子,会转)   = 中央 φ9 短凸台(空心轴) + 3×M3孔(品字形,相邻10mm) —— 用户实测

 实现方式：CSG 布尔（trimesh + manifold3d），实心盘 - 孔圆柱 = 真·通孔。

 依赖： pip install trimesh manifold3d shapely
 用法： python generate_gimbal_stl.py
 输出： ./stl/arm.stl  ./stl/clamp.stl
=====================================================================
"""

import math
import os

import numpy as np
import trimesh

# ================= 电机关键尺寸（mm）=================
THICK = 5.0           # 板厚

# —— 转子顶部接口（套转子端盖：φ9 凸台 + 3×M3 品字形）——
TOP_BOSS_R = 4.8      # 中央凸台定位孔半径（φ9.6，套 φ9 凸台留 0.3mm 余量）
TOP_SCREW_R = 1.7     # M3 过孔半径（φ3.4）
_SCW = 10.0 / math.sqrt(3.0)   # 品字形等边三角形外接圆半径 ≈ 5.7735
ROTOR_HOLES = [              # 3 孔品字形坐标（相邻中心距 10mm，用户实测）
    (_SCW, 0.0),
    (-_SCW * 0.5, 5.0),
    (-_SCW * 0.5, -5.0),
]

# —— 定子接口（套定子底座：4×M3 铜螺柱正方形 + 中央小孔避开固定螺丝头）——
STATOR_HOLE_R = 4.0   # 中央小孔（φ8，仅避开中央那颗固定螺丝头 φ4~5，不碰 4 个 M3 孔）
MOUNT_OFF = 10.5      # 4×M3 孔偏移（±10.5 正方形，相邻 21 / 对角 30，用户实测）

# —— L 臂布局 ——
ARM_X = 40.0          # 电机B轴心相对电机A轴心的水平距离
ARM_Z = 30.0          # 电机B轴心相对电机A转子顶面的高度
HORIZ_R = 20.0        # 水平盘外半径（φ40）
VERT_R = 22.0         # 竖直盘 / 夹座安装盘外半径（φ44）

# —— 夹座布局（clamp 套电机B转子顶部，B 轴沿 y，B 定子在 arm 竖直盘 y=0 处）——
MOTOR_T = 24.0        # 电机厚度（转子顶面相对定子面的距离，仅用于预览定位）
CLAMP_Y = MOTOR_T     # 夹座安装盘中心 y 坐标（= 电机厚度，即 B 转子顶部位置）

# —— 笔夹套筒（沿 +x 伸出，φ8 通孔夹圆珠笔）——
PEN_INNER_R = 4.0     # 笔孔半径（φ8，用户指定，夹正常圆珠笔）
PEN_OUTER_R = 9.0     # 笔套筒外径（φ18，壁厚 5mm）
PEN_LEN = 30.0        # 笔套筒长度（沿 x）
PEN_CX = 68.0         # 笔套筒中心 x（套筒 x[53,83]，与安装盘右边缘重叠 9mm）

# —— 白色供电插座避让（arm 竖直盘 +x 方向开 U 形缺口，让插座和导线穿出）——
# 用户实测插座：长边 13mm（径向凸出）× 短边 10mm（切向宽）
NOTCH_W = 12.0        # 缺口宽度（z 方向，容纳 10mm 短边 + 2mm 余量）
NOTCH_D = 14.0        # 缺口深度（x 方向，容纳 13mm 长边 + 余量）
NOTCH_IN = 12.0       # 缺口内缘离竖直盘中心的 x 距离（插座根部在 PCB 边缘 x≈55 处）

# 布尔运算引擎
ENGINE = 'manifold'


def _cyl(radius, height, cx=0.0, cy=0.0, cz=0.0, sections=64):
    """沿 z 轴的圆柱，平移到 (cx, cy, cz)。"""
    m = trimesh.creation.cylinder(radius=radius, height=height, sections=sections)
    m.apply_translation([cx, cy, cz])
    return m


def _box(sx, sy, sz, cx=0.0, cy=0.0, cz=0.0):
    m = trimesh.creation.box(extents=[sx, sy, sz])
    m.apply_translation([cx, cy, cz])
    return m


def _rotor_face(outer_r, thickness):
    """「套转子顶部」圆盘：φ9.6 中央孔 + 3×M3 品字形。截面 xy、法向 z、中心在原点。"""
    plate = _cyl(outer_r, thickness, cz=0.0)
    boss = _cyl(TOP_BOSS_R, thickness + 2.0, cz=0.0)
    screws = [_cyl(TOP_SCREW_R, thickness + 2.0, cx=hx, cy=hy, cz=0.0, sections=48)
              for hx, hy in ROTOR_HOLES]
    return trimesh.boolean.difference([plate, boss] + screws, engine=ENGINE)


def _stator_face(outer_r, thickness):
    """「套定子底座」圆盘：中央 φ8 小孔(避开螺丝头) + 4×M3 正方形。截面 xy、法向 z、中心在原点。"""
    plate = _cyl(outer_r, thickness, cz=0.0)
    stator = _cyl(STATOR_HOLE_R, thickness + 2.0, cz=0.0)
    screws = [_cyl(TOP_SCREW_R, thickness + 2.0, cx=sx, cy=sz, cz=0.0, sections=48)
              for sx in (-MOUNT_OFF, MOUNT_OFF) for sz in (-MOUNT_OFF, MOUNT_OFF)]
    return trimesh.boolean.difference([plate, stator] + screws, engine=ENGINE)


def _rot_x(angle):
    """绕 x 轴旋转矩阵：把 z 轴转向 y 轴（截面 xy 平面 → xz 平面）。"""
    return trimesh.transformations.rotation_matrix(angle, [1, 0, 0])


def _cyl_x(radius, height, cx, cy, cz, sections=64):
    """沿 x 轴的圆柱（绕 y 轴转 90°），平移到 (cx, cy, cz)。"""
    m = trimesh.creation.cylinder(radius=radius, height=height, sections=sections)
    m.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2.0, [0, 1, 0]))
    m.apply_translation([cx, cy, cz])
    return m


def build_arm():
    """L 型连接臂：水平盘套A转子(偏航) + 连接块 + 竖直盘固定B定子(俯仰固定端)。"""

    # 1) 水平盘（xy 平面，z 0~5）：套电机A转子顶部
    horizontal = _rotor_face(HORIZ_R, THICK)
    horizontal.apply_translation([0.0, 0.0, THICK / 2.0])

    # 2) 竖直盘（xz 平面，中心 (ARM_X,0,ARM_Z)）：固定电机B定子（φ8中央孔+4×M3）
    vertical = _stator_face(VERT_R, THICK)
    vertical.apply_transform(_rot_x(math.pi / 2.0))
    vertical.apply_translation([ARM_X, 0.0, ARM_Z])

    # 2b) 白色供电插座避让缺口：竖直盘 +x 方向开 U 形通槽，让插座+导线穿出
    notch = _box(NOTCH_D, THICK + 4.0, NOTCH_W,
                 cx=ARM_X + NOTCH_IN + NOTCH_D / 2.0, cy=0.0, cz=ARM_Z)
    vertical = trimesh.boolean.difference([vertical, notch], engine=ENGINE)

    # 3) 连接块（实心，连接两盘，避开所有孔）
    block = _box(10.0, 16.0, 30.0, cx=20.0, cy=0.0, cz=15.0)

    # 4) 合并
    return trimesh.boolean.union([horizontal, vertical, block], engine=ENGINE)


def build_clamp():
    """夹笔座：安装盘套B转子顶部(φ9.6+3×M3) + φ8 通孔套筒夹圆珠笔(笔沿 x)。"""

    # 1) 安装盘（xz 平面，中心 (ARM_X, CLAMP_Y, ARM_Z)）：套电机B转子顶部
    board = _rotor_face(VERT_R, THICK)
    board.apply_transform(_rot_x(math.pi / 2.0))
    board.apply_translation([ARM_X, CLAMP_Y, ARM_Z])

    # 2) 笔夹套筒（沿 +x 伸出，φ8 通孔夹笔；外径 φ18，与安装盘右边缘重叠连接）
    sleeve = _cyl_x(PEN_OUTER_R, PEN_LEN, cx=PEN_CX, cy=CLAMP_Y, cz=ARM_Z)
    bore = _cyl_x(PEN_INNER_R, PEN_LEN + 6.0, cx=PEN_CX, cy=CLAMP_Y, cz=ARM_Z)

    body = trimesh.boolean.union([board, sleeve], engine=ENGINE)
    return trimesh.boolean.difference([body, bore], engine=ENGINE)


def verify():
    print("=== 校验：转子 3 孔品字形中心距 ===")
    for i in range(3):
        a = ROTOR_HOLES[i]
        b = ROTOR_HOLES[(i + 1) % 3]
        d = math.hypot(a[0] - b[0], a[1] - b[1])
        print(f"  孔{i}->孔{(i + 1) % 3}: {d:.3f} mm (应为 10.000)")


def report(name, mesh):
    b = mesh.bounds
    print(f"[{name}] 水密={mesh.is_watertight}  体积={mesh.volume:.0f} mm^3  "
          f"三角形={len(mesh.faces)}")
    print(f"       尺寸 X[{b[0][0]:.1f},{b[1][0]:.1f}] "
          f"Y[{b[0][1]:.1f},{b[1][1]:.1f}] Z[{b[0][2]:.1f},{b[1][2]:.1f}]")


def main():
    outdir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'stl')
    os.makedirs(outdir, exist_ok=True)

    verify()

    arm = build_arm()
    report('arm', arm)
    arm.export(os.path.join(outdir, 'arm.stl'))

    clamp = build_clamp()
    report('clamp', clamp)
    clamp.export(os.path.join(outdir, 'clamp.stl'))

    print("\n生成完成： ./stl/arm.stl  ./stl/clamp.stl")


if __name__ == '__main__':
    main()
