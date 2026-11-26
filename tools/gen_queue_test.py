# -*- coding: utf-8 -*-
"""
生成双轴测试队列（q 命令）并计算总时长。

复刻 lib/MYFOC/foc_trajectory.cpp 的 traj_begin 规划：
  7 段 S 曲线，T = 2*(2Tj + Ta) + Tc
  A = TRAJ_ACCEL = 30 rad/s², J = TRAJ_JERK = 120 rad/s³
  队列里写的 speed 是 vmax（rad/s）

输出可直接粘进串口助手的 q 命令字符串。
"""
import math

A = 30.0   # rad/s^2
J = 120.0  # rad/s^3
QUEUE_SIZE = 64


def seg_time(dist_rad, vmax):
    """单段轨迹时长（秒），与固件 traj_begin 等价"""
    D = abs(dist_rad)
    if D <= 1e-4:
        return 0.0
    V = vmax if vmax > 0 else A

    # 步骤1：能否升到 vmax
    if V * J <= A * A:
        apk = math.sqrt(V * J)
        Tj = math.sqrt(V / J)
        Ta = 0.0
    else:
        apk = A
        Tj = A / J
        Ta = V / A - Tj
        if Ta < 0:
            Ta = 0.0

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

    return 2.0 * (2.0 * Tj + Ta) + Tc


def build(name, pairs, vmax, note=""):
    """
    pairs: [(m0_deg, m1_deg), ...] 每段的两个轴目标（度，对外坐标）
    生成交替写两轴的队列，保证每条段两轴各一条 → 两轴段数相同、时长相同
    """
    parts = []
    total_t = 0.0
    for m0, m1 in pairs:
        parts.append("0,%g,%g" % (m0, vmax))
        parts.append("1,%g,%g" % (m1, vmax))
        # 每段的时间由该轴自己的距离决定（两轴距离通常相同）
        total_t += max(seg_time(math.radians(m0 - prev[0]), vmax) if prev else 0 for prev in [None]) if False else 0
    # 时长另算（用绝对目标序列算距离）
    q = "q" + ";".join(parts)
    return q


def route(name, points, vmax, note=""):
    """
    points: [((m0,m1), (m0,m1), ...] 绝对目标序列，第一点是起点
    返回 (队列字符串, 理论总时长)
    """
    segs = []
    t_total = 0.0
    for i in range(1, len(points)):
        p0_prev, p1_prev = points[i - 1]
        p0, p1 = points[i]
        segs.append("0,%g,%g" % (p0, vmax))
        segs.append("1,%g,%g" % (p1, vmax))
        d0 = math.radians(p0 - p0_prev)
        d1 = math.radians(p1 - p1_prev)
        # 两轴取较长时间（慢的那条决定这一段何时两轴都到位）
        t_total += max(seg_time(d0, vmax), seg_time(d1, vmax))
    return "q" + ";".join(segs), t_total, len(segs)


def main():
    print("=" * 74)
    print("单段时长 vs 速度（行程 170°，用于选速度）")
    print("=" * 74)
    print("  vmax(rad/s)  约合(deg/s)   单段(170°)时长")
    for v in (0.3, 0.5, 0.7, 1.0, 1.5, 2.0):
        t = seg_time(math.radians(170), v)
        print("   %5.2f       %6.1f        %5.2f s" % (v, v * 57.2958, t))

    print()
    print("=" * 74)
    print("组 1：反向对扫  —— 两轴同时动、方向相反（推荐先跑）")
    print("=" * 74)
    pts = [((5, 5), (5, 5))]
    A0, A1 = 5, 175
    B0, B1 = 175, 5
    for i in range(8):
        pts.append(((A0, A1) if i % 2 == 0 else (B0, B1)))
    # 上面写法把两个轴一起写，重排：
    pts = [(5, 5)]
    for i in range(16):
        if i % 2 == 0:
            pts.append((175, 5))     # M0 往上、M1 往下
        else:
            pts.append((5, 175))     # M0 往下、M1 往上
    q1, t1, n1 = route("rev", pts, 1.0)
    print("段数 = %d（每轴），理论总时长 ≈ %.1f s" % (n1 // 2, t1))
    print(q1)

    print()
    print("=" * 74)
    print("组 2：同向对扫 —— 两轴同方向（看两轴是否一致）")
    print("=" * 74)
    pts = [(5, 5)]
    for i in range(16):
        pts.append((175, 175) if i % 2 == 0 else (5, 5))
    q2, t2, n2 = route("same", pts, 1.0)
    print("段数 = %d（每轴），理论总时长 ≈ %.1f s" % (n2 // 2, t2))
    print(q2)

    print()
    print("=" * 74)
    print("组 3：慢速反向 60 秒 —— 长时间稳定性/发热/漂移观察")
    print("=" * 74)
    pts = [(5, 5)]
    for i in range(10):
        pts.append((175, 5) if i % 2 == 0 else (5, 175))
    q3, t3, n3 = route("slow", pts, 0.5)
    print("段数 = %d（每轴），理论总时长 ≈ %.1f s" % (n3 // 2, t3))
    print(q3)

    print()
    print("=" * 74)
    print("校验")
    print("=" * 74)
    for tag, q, t in (("组1", q1, t1), ("组2", q2, t2), ("组3", q3, t3)):
        nseg_m0 = q.count("0,")
        nseg_m1 = q.count("1,")
        ok_len = len(q) < 2048
        ok_q = nseg_m0 <= QUEUE_SIZE and nseg_m1 <= QUEUE_SIZE
        print("  %s  串口长度=%4d 字符 %s   m0段=%d m1段=%d %s   总时长 %.1fs"
              % (tag, len(q), "OK" if ok_len else "超长!",
                 nseg_m0, nseg_m1, "OK" if ok_q else "超队列!",
                 t))


if __name__ == "__main__":
    main()
