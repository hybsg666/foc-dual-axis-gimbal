"""I2C 稳定性统计：静置 N 秒，统计 I2C 错误行、resync、vel spike 数量。

用法：python tools/i2cstat.py --sec 10
"""
import argparse
import time

import serial


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM3")
    ap.add_argument("--sec", type=float, default=10)
    a = ap.parse_args()

    s = serial.Serial()
    s.port = a.port
    s.baudrate = 115200
    s.timeout = 0.2
    s.dtr = False
    s.rts = False
    s.open()
    time.sleep(0.3)
    s.reset_input_buffer()

    t0 = time.time()
    err = resync = spike = buserr = 0
    other = []
    while time.time() - t0 < a.sec:
        line = s.readline().decode("utf8", "replace").strip()
        if not line:
            continue
        if "Error 263" in line or "Error -1" in line:
            err += 1
        elif "resync" in line:
            resync += 1
            other.append(line)
        elif "vel spike" in line:
            spike += 1
        elif "bus error" in line:
            buserr += 1
            other.append(line)
        elif not line[0].isdigit():
            other.append(line)
    s.close()
    print("%.0fs 内：I2C错误行 %d  总线恢复 %d  resync %d  vel spike %d"
          % (a.sec, err, buserr, resync, spike))
    for l in other[:10]:
        print("  " + l[:140])


if __name__ == "__main__":
    main()
