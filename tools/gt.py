#!/usr/bin/env python3
"""gt.py - 云台串口一键测试/采样工具

用法:
  python tools/gt.py send "0r" "1r" "?"
  python tools/gt.py log 5            # 抓 5 秒原始日志
  python tools/gt.py seq "0r" -w 0.3 "?" -w 1.0
"""
import sys, time, argparse
import serial

PORT = "COM3"
BAUD = 115200

def open_port(port=PORT, baud=BAUD, reset=False):
    s = serial.Serial()
    s.port = port
    s.baudrate = baud
    s.timeout = 0.05
    # ★ dtr/rts 必须在 open() 之前设，否则 ESP32 的自动复位电路会在打开瞬间
    #   把 EN 拉低 —— 每次连串口都重启一次，命令全丢在启动期。踩过。
    s.dtr = False
    s.rts = False
    s.open()
    if reset:
        s.dtr = True; time.sleep(0.1); s.dtr = False
    return s

def drain(s, sec):
    out = []
    t0 = time.time()
    while time.time() - t0 < sec:
        line = s.readline()
        if line:
            out.append(line.decode("utf-8", "replace").rstrip("\r\n"))
    return out

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", choices=["send", "log", "seq"])
    ap.add_argument("items", nargs="*")
    ap.add_argument("-t", "--time", type=float, default=1.5, help="每条命令后采集秒数")
    ap.add_argument("-p", "--port", default=PORT)
    args = ap.parse_args()

    s = open_port(args.port)
    drain(s, 0.3)   # 清掉旧数据

    if args.mode == "log":
        sec = float(args.items[0]) if args.items else 3.0
        for ln in drain(s, sec):
            print(ln)
        s.close(); return

    if args.mode == "send":
        for cmd in args.items:
            s.write((cmd + "\n").encode())
            s.flush()
            for ln in drain(s, args.time):
                print(ln)
        s.close(); return

    if args.mode == "seq":
        i = 0
        wait = args.time
        while i < len(args.items):
            it = args.items[i]
            if it == "-w":
                wait = float(args.items[i + 1]); i += 2; continue
            s.write((it + "\n").encode()); s.flush()
            print(f"### >>> {it}")
            for ln in drain(s, wait):
                print(ln)
            i += 1
        s.close(); return

if __name__ == "__main__":
    main()
