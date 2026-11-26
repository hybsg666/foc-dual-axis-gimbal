# -*- coding: utf-8 -*-
"""
=====================================================================
 位置模式一键诊断（故障码 4 / 转不到位）
=====================================================================
 它做的事：
   1. 打开串口
   2. 依次发  1r  →  1p0  →  1p1.57
   3. 边采边解析遥测，每 200ms 打一行摘要
   4. 结尾自动给结论：是"根本没动"、"转了一半"、"抖动"还是"到位"

 用法（在 3MytowFOC SVPWM 目录下）：
   python tools/diag_pos.py                 # 自动找串口，默认测电机 1 到 1.57
   python tools/diag_pos.py -p COM5         # 指定串口
   python tools/diag_pos.py -m 0            # 测电机 0
   python tools/diag_pos.py -t 0.785        # 目标改成 45°

 输出请整段贴回给我。
=====================================================================
"""

import argparse
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("缺少 pyserial。请在 Anaconda Prompt 里执行： pip install pyserial")
    sys.exit(1)

COLS = ["mode", "target", "pos", "vel", "iq", "iqcmd", "fault"]
ROWS = ["目标", "实际位置", "速度", "电流iq", "电流指令", "故障码"]


def pick_port(user_port):
    if user_port:
        return user_port
    ports = list(list_ports.comports())
    if not ports:
        print("没找到任何串口。检查 USB 线、驱动，或用 -p COM5 指定。")
        sys.exit(1)
    if len(ports) == 1:
        return ports[0].device
    print("检测到多个串口：")
    for i, p in enumerate(ports):
        print("  [%d] %s  %s" % (i, p.device, p.description))
    idx = input("输入编号: ").strip()
    return ports[int(idx)].device


def parse_telemetry(line):
    """遥测一行 14 列逗号分隔：前 7 列电机0，后 7 列电机1。"""
    parts = line.split(",")
    if len(parts) != 14:
        return None
    try:
        vals = [float(x) for x in parts]
    except ValueError:
        return None
    return vals


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-p", "--port", default=None, help="串口号，如 COM5")
    ap.add_argument("-m", "--motor", type=int, default=1, help="测哪台电机 0/1")
    ap.add_argument("-t", "--target", type=float, default=1.57, help="目标位置 rad")
    ap.add_argument("-s", "--seconds", type=float, default=8.0, help="到位后观察秒数")
    args = ap.parse_args()

    port = pick_port(args.port)
    print("串口: %s   电机: M%d   目标: %.3f rad (%.1f deg)\n"
          % (port, args.motor, args.target, args.target * 57.2958))

    ser = serial.Serial(port, 115200, timeout=0.05)
    time.sleep(0.3)
    ser.reset_input_buffer()

    m = args.motor
    base = m * 7

    def send(cmd):
        ser.write((cmd + "\n").encode())
        print(">>> %s" % cmd)

    def pump(seconds, show=False):
        """读串口，解析遥测；show=True 时每 200ms 打一行摘要。"""
        t0 = time.time()
        last_show = 0.0
        last = None
        events = []
        while time.time() - t0 < seconds:
            raw = ser.readline().decode("utf-8", "ignore").strip()
            if not raw:
                continue
            if not raw[0].isdigit():
                # 固件打印的文本行（[main]/[STUCK]/[ENC]/[FAULT]）
                if raw.startswith("["):
                    print("   FB: %s" % raw)
                    events.append(raw)
                continue
            v = parse_telemetry(raw)
            if v is None:
                continue
            last = v[base:base + 7]
            if show and time.time() - last_show > 0.2:
                last_show = time.time()
                print("   t=%4.1fs  pos=%+.3f  vel=%+.3f  iq=%+.3f  iqcmd=%+.3f  fault=%d"
                      % (time.time() - t0, last[2], last[3], last[4], last[5], int(last[6])))
        return last, events

    # --- 1. 清故障 ---
    send("%dr" % m)
    time.sleep(0.4)

    # --- 2. 回零 ---
    print("\n[1/2] 回零 ...")
    send("%dp0" % m)
    pump(2.5, show=False)
    ser.reset_input_buffer()
    pump(0.3, show=True)

    # --- 3. 走目标 ---
    print("\n[2/2] 走 %.3f rad，观察 %.0f 秒 ..." % (args.target, args.seconds))
    send("%dp%.4f" % (m, args.target))
    last, events = pump(args.seconds, show=True)

    ser.close()

    # ---------------- 结论 ----------------
    print("\n================ 结论 ================")
    if last is None:
        print("没收到任何遥测数据。检查：串口是否选错 / 固件是否在跑 / 遥测是否被 'd' 命令关掉了。")
        return

    mode, target, pos, vel, iq, iqcmd, fault = last
    print("最后状态: mode=%d target=%.3f pos=%.3f vel=%.3f iq=%.3f iqcmd=%.3f fault=%d"
          % (mode, target, pos, vel, iq, iqcmd, int(fault)))

    err = target - pos
    if abs(err) < 0.1:
        print("✅ 到位了（误差 %.3f rad）" % err)
    elif abs(pos) < 0.05 and abs(target) > 0.3:
        print("❌ 根本没动。")
        if abs(iq) < 0.05:
            print("   → iq 几乎为 0：控制没给力。可能是电流采样/驱动使能/电源问题。")
        elif abs(iqcmd) > 0.3:
            print("   → iqcmd 很大但 iq 很小：电流环被卡住，查 ADC 采样与驱动供电。")
        else:
            print("   → iqcmd 本身就小：位置环/速度环没输出，查编码器读数是否冻结。")
    elif abs(err) > 0.3:
        print("❌ 只走到 %.1f%%，卡在半路。" % (abs(pos) / abs(target) * 100))
        if abs(iq) > 0.4:
            print("   → iq 顶到限幅：力矩不够，或真的被机械/线缆挡住。")
            print("     先确认转子上的东西没碰到桌面、线没绷紧。")
        else:
            print("   → iq 很小却卡住：不是力矩问题，是控制/编码器问题。")
    else:
        print("⚠️  接近但没完全到位（误差 %.3f rad）" % err)

    if fault:
        print("故障码 = %d" % int(fault))
    if events:
        print("\n关键固件日志：")
        for e in events:
            print("  " + e)


if __name__ == "__main__":
    main()
