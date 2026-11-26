#!/usr/bin/env python3
"""tune.py - 单变量扫描：改一个参数 → 跑同一段定点跳跃 → 量落地后稳态抖动

用法:
  python tools/tune.py --sweep "k0.20,0.30" "k0.12,0.30" "k0.12,1.50"
  python tools/tune.py --sweep "w0.12" "w0.25"

每组流程：设参数 → 回 45° 起点 → 片上采集 2.8s 的 45↔128 跳跃 → 取尾段指标
指标口径：尾段最后 0.4s 的位置峰峰 = 稳态抖动幅度（"观感抖不抖"）
"""
import sys, time, argparse
import serial

PORT = "COM3"

def open_port():
    s = serial.Serial()
    s.port = PORT; s.baudrate = 115200; s.timeout = 0.1
    s.dtr = False; s.rts = False     # ★ open() 前设，否则复位板子
    s.open()
    time.sleep(0.15)
    s.reset_input_buffer()
    return s

def send(s, cmd, wait):
    s.write((cmd + "\n").encode()); s.flush()
    t0 = time.time()
    while time.time() - t0 < wait:
        s.readline()

def capture(s, cmd, ms=2800):
    s.write(f"0o{ms}\n".encode()); s.flush()
    time.sleep(0.05)
    s.write((cmd + "\n").encode()); s.flush()
    wave, tail = [], {}
    t0 = time.time()
    while time.time() - t0 < ms / 1000.0 + 5.0:
        line = s.readline().decode("utf-8", "replace").strip()
        if not line: continue
        if line.startswith("[WAVE]"):
            if line.startswith("[WAVE] P:") or line.startswith("[WAVE] V:") \
               or line.startswith("[WAVE] C:"):
                tail[line[7]] = [float(x) for x in line[9:].split()]
            else:
                wave.append(line)
        if len(tail) == 3: break
    return wave, tail

def metrics(tail):
    p = tail.get("P", [])
    v = tail.get("V", [])
    if len(p) < 60: return None
    seg = p[-40:]                     # 最后 0.4s
    vs  = v[-40:] if len(v) >= 40 else v
    return {
        "pp_tail": max(seg) - min(seg),
        "pp_all":  max(p) - min(p),
        "v_tail":  max(vs) - min(vs),
        "mean":    sum(seg) / len(seg),
        "lo":      min(seg), "hi": max(seg),
        # 全程最大过冲：相对末段均值
        "overshoot": max(abs(x - (sum(seg)/len(seg))) for x in p),
    }

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sweep", nargs="+", required=True)
    ap.add_argument("--cmd", default="0b45@200,128@8000;450")
    ap.add_argument("--ms", type=int, default=2500)
    ap.add_argument("--start", default="0p45")
    a = ap.parse_args()

    s = open_port()
    print(f"{'参数':<16}{'尾段峰峰':>10}{'瞬态峰值':>10}{'速度峰峰':>10}{'稳态中心':>10}")
    print("-" * 58)
    for pset in a.sweep:
        # pset 可以是组合："k0.10,0.30|w0.30" 用 | 分隔多条设置命令
        for one in pset.split("|"):
            send(s, "0" + one.strip(), 0.35)
        send(s, a.start, 2.0)             # 回起点并等稳
        wave, tail = capture(s, a.cmd, a.ms)
        m = metrics(tail)
        if not m:
            print(f"{pset:<16}  采集失败 {wave}")
            continue
        print(f"{pset:<16}{m['pp_tail']:>9.2f}°{m['overshoot']:>9.2f}°"
              f"{m['v_tail']:>9.0f}°/s{m['mean']:>9.2f}°")
    send(s, "0p45", 0.5)
    s.close()

if __name__ == "__main__":
    main()
