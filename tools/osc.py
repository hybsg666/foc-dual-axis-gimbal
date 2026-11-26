#!/usr/bin/env python3
"""osc.py - 片上示波器：启动 'o' 采集后立刻发运动命令，回收 [WAVE] 报告

用法:
  python tools/osc.py "0b128@1200,45@1200;450"          # 默认 2800ms 采集
  python tools/osc.py "0b128@1200,45@1200;450" -m 3000
  python tools/osc.py "0b128@1200,45@1200;450" -a 1     # 采 M1
"""
import sys, time, argparse
import serial

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd")
    ap.add_argument("-m", "--ms", type=int, default=2800)
    ap.add_argument("-a", "--axis", type=int, default=0)
    ap.add_argument("-p", "--port", default="COM3")
    a = ap.parse_args()

    s = serial.Serial()
    s.port = a.port; s.baudrate = 115200; s.timeout = 0.1
    s.dtr = False; s.rts = False      # ★ 必须在 open() 前，否则会复位板子
    s.open()
    time.sleep(0.15); s.reset_input_buffer()

    s.write(f"{a.axis}o{a.ms}\n".encode()); s.flush()
    time.sleep(0.05)
    s.write((a.cmd + "\n").encode()); s.flush()

    # 采集 ms + 打印时间
    t0 = time.time()
    limit = a.ms / 1000.0 + 4.0
    buf = ""
    tail = {}
    while time.time() - t0 < limit:
        chunk = s.readline().decode("utf-8", "replace")
        if not chunk: continue
        line = chunk.strip()
        if line.startswith("[WAVE]"):
            print(line)
            if line.startswith("[WAVE] P:") or line.startswith("[WAVE] V:") \
               or line.startswith("[WAVE] C:"):
                tail[line[7]] = [float(x) for x in line[9:].split()]
        buf += line
        if len(tail) == 3:
            break
    s.close()

    # 尾段（最后 0.5s）峰峰 = 稳态抖动幅度，这是"观感抖不抖"的直接指标
    if "P" in tail and len(tail["P"]) > 60:
        p = tail["P"]
        for name, seg in (("全段", p), ("尾0.5s", p[-50:])):
            print(f"    {name}: 峰峰 {max(seg)-min(seg):.2f}°  "
                  f"范围 [{min(seg):.2f}, {max(seg):.2f}]")
    if "V" in tail and len(tail["V"]) > 60:
        v = tail["V"]
        print(f"    速度尾段: 峰峰 {max(v[-50:])-min(v[-50:]):.0f}°/s")

if __name__ == "__main__":
    main()
