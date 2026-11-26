# -*- coding: utf-8 -*-
"""
=====================================================================
 连续扫描（让电机真正"一直动"）
=====================================================================
 为什么要这个工具：
   串口一条 q 命令最多带 64 条轨迹，跑完就停——这是设计如此，不是坏了。
   想一直摆动，就得"跑完一批立刻补下一批"。这个脚本干这件事。

 用法（在 3MytowFOC SVPWM 目录下）：
   python tools/scan_loop.py                        # M1，0° ↔ 90°，5 rad/s，一直扫到 Ctrl+C
   python tools/scan_loop.py -m 0 -a -45 -b 45 -v 4
   python tools/scan_loop.py -n 4                   # 只扫 4 批（约 1 分钟）后退出

 参数：
   -a -b  两个端点（单位：度）
   -v     速度 rad/s
   -n     跑几批（不填 = 无限，Ctrl+C 停）

 每批 20 段（19 次运动，约 14 秒），脚本靠固件打印的
 "[QUEUE] m1 20 segs done" 判断这批结束了，再发下一批，不会打断动作。
=====================================================================
"""

import argparse
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("缺少 pyserial。请执行： pip install pyserial")
    sys.exit(1)

RAD = 3.141592653589793 / 180.0


def pick_port(user_port):
    if user_port:
        return user_port
    ports = list(list_ports.comports())
    if not ports:
        print("没找到串口。")
        sys.exit(1)
    if len(ports) == 1:
        return ports[0].device
    print("检测到多个串口：")
    for i, p in enumerate(ports):
        print("  [%d] %s  %s" % (i, p.device, p.description))
    return ports[int(input("输入编号: ").strip())].device


def build_cmd(motor, lo_rad, hi_rad, speed, segs):
    """构造 q 命令：从 lo 出发，在 lo/hi 之间往复 segs 条"""
    pts = [lo_rad]
    for i in range(1, segs):
        pts.append(hi_rad if i % 2 else lo_rad)
    return "q" + ";".join("%d,%.4f,%.2f" % (motor, p, speed) for p in pts)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-p", "--port", default=None)
    ap.add_argument("-m", "--motor", type=int, default=1)
    ap.add_argument("-a", "--lo", type=float, default=0.0, help="端点A（度）")
    ap.add_argument("-b", "--hi", type=float, default=90.0, help="端点B（度）")
    ap.add_argument("-v", "--speed", type=float, default=5.0)
    ap.add_argument("-n", "--batches", type=int, default=0, help="跑几批，0=无限")
    ap.add_argument("--segs", type=int, default=20, help="每批几条轨迹")
    args = ap.parse_args()

    port = pick_port(args.port)
    lo, hi = args.lo * RAD, args.hi * RAD
    print("串口 %s   M%d   %.0f° ↔ %.0f°   速度 %.1f rad/s   每批 %d 条"
          % (port, args.motor, args.lo, args.hi, args.speed, args.segs))
    print("Ctrl+C 停止\n")

    ser = serial.Serial(port, 115200, timeout=0.1)
    time.sleep(0.3)
    ser.reset_input_buffer()

    cmd = build_cmd(args.motor, lo, hi, args.speed, args.segs)
    done_tag = "[QUEUE] m%d %d segs done" % (args.motor, args.segs)

    batch = 0
    t0 = time.time()
    try:
        while True:
            batch += 1
            ser.reset_input_buffer()
            ser.write((cmd + "\n").encode())
            print("[批 %d] 已发送 %d 条，等待执行完 ..." % (batch, args.segs))

            t_send = time.time()
            finished = False
            while time.time() - t_send < 60:
                raw = ser.readline().decode("utf-8", "ignore").strip()
                if not raw:
                    continue
                if raw.startswith("["):
                    if raw.startswith("[QUEUE] m%d" % args.motor):
                        print("   %s  (%.1fs)" % (raw, time.time() - t_send))
                        finished = True
                        break
                    if raw.startswith("[LIMIT]") or raw.startswith("[FAULT]"):
                        print("   !! %s" % raw)
                elif raw[0].isdigit():
                    continue
            if not finished:
                print("   [警告] 60s 内没等到完成信号，可能中途出错，继续发下一批")

            if args.batches and batch >= args.batches:
                break
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\n已停止。")
    finally:
        ser.close()
        print("共跑了 %d 批，用时 %.1f 秒" % (batch, time.time() - t0))


if __name__ == "__main__":
    main()
