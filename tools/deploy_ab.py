"""一键：编译 -> 烧录 -> 验证版本 -> 到位成形器 A/B 测试。

用法（在项目根目录 D:/FOC/MyFOCProject/4YT 下）：
    python tools/deploy_ab.py                 # 全自动：找口 + 烧录 + A/B
    python tools/deploy_ab.py --port COM3     # 指定串口
    python tools/deploy_ab.py --no-upload     # 跳过烧录，只跑测试
    python tools/deploy_ab.py --rep 3         # 每档测 3 次（更稳，更慢）

跑完会打印一张表 + 推荐档位，并把原始结果存到 runs/settle_ab_<时间>.txt。
"""
import argparse, os, statistics, subprocess, sys, time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("[!] 当前 python 没有 pyserial，正在自动安装 ...")
    subprocess.run([sys.executable, "-m", "pip", "install", "pyserial", "-q"])
    try:
        import serial
        import serial.tools.list_ports
    except ImportError:
        print("[X] 装不上。请手动执行：  %s -m pip install pyserial" % sys.executable)
        print("    或改用 Anaconda Prompt 里的 python 跑这个脚本。")
        sys.exit(1)


def find_port(explicit=None):
    if explicit:
        return explicit
    cands = []
    for p in serial.tools.list_ports.comports():
        d = (p.description or "").upper()
        hw = (p.hwid or "").upper()
        if "BTHENUM" in hw or "BLUETOOTH" in d or "蓝牙" in d:
            continue
        if "CH343" in d or "CH340" in d or "CP210" in d or "1A86" in hw:
            cands.append(p.device)
    if cands:
        return cands[0]
    allp = [p.device for p in serial.tools.list_ports.comports()]
    if len(allp) == 1:
        return allp[0]
    return None


def find_pio():
    import shutil
    w = shutil.which("pio") or shutil.which("platformio")
    if w:
        return w
    home = os.path.expanduser("~")
    for c in (os.path.join(home, ".platformio", "penv", "Scripts", "pio.exe"),
              os.path.join(home, ".platformio", "penv", "bin", "pio")):
        if os.path.exists(c):
            return c
    return None


def upload():
    pio = find_pio()
    if not pio:
        print("[X] 找不到 pio。请在 VSCode PlatformIO 里点 Upload，或把 pio 加进 PATH。")
        return False
    print(">>> 编译 + 烧录中（约 30~60 秒，别拔线）...")
    # ★ 不用 capture_output / text（Python 3.7+ 才有）；Anaconda 是 3.6
    r = subprocess.run([pio, "run", "-e", "esp32-s3-devkitc-1", "-t", "upload"],
                       cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    out = (r.stdout or b"").decode("utf-8", "replace")
    if r.returncode != 0 or "SUCCESS" not in out:
        print("[X] 烧录失败，原始输出末尾：")
        print(out[-1500:])
        return False
    for line in out.splitlines():
        if "SUCCESS" in line or "RAM:" in line or "Flash:" in line:
            print("   ", line.strip())
    print("[OK] 烧录完成")
    return True


def open_port(port):
    s = serial.Serial()
    s.port = port
    s.baudrate = 115200
    s.timeout = 0.2
    s.dtr = False      # ★ 必须在 open() 之前，否则每次连串口都会复位板子
    s.rts = False
    s.open()
    time.sleep(0.4)
    s.reset_input_buffer()
    return s


def wait_ready(s, tmo=12.0):
    """读启动横幅，确认固件版本；顺带确认板子活着。"""
    ver = None
    t0 = time.time()
    while time.time() - t0 < tmo:
        try:
            line = s.readline().decode("utf8", "replace").strip()
        except Exception:
            continue
        if "FW-BUILD" in line:
            ver = line
            print("[版本]", line)
            break
    if ver is None:
        print("[!] 没看到 FW-BUILD 横幅（板子可能已经启动完了，正常，继续）")
    return ver


def send(s, cmd, w=0.4):
    s.write((cmd + "\n").encode())
    s.flush()
    t = time.time()
    while time.time() - t < w:
        s.readline()


def grab(s, ms):
    P = V = C = I = None
    t0 = time.time()
    while time.time() - t0 < 10:
        l = s.readline().decode("utf8", "replace").strip()
        if l.startswith("[WAVE] P:"):
            P = [float(x) for x in l[9:].split()]
        elif l.startswith("[WAVE] V:"):
            V = [float(x) for x in l[9:].split()]
        elif l.startswith("[WAVE] C:"):
            C = [float(x) for x in l[9:].split()]
        elif l.startswith("[WAVE] I:"):
            I = [float(x) for x in l[9:].split()]
        if P and V and I:
            return P, V, I
    return None, None, None


def trial(s, target, ms=1200):
    send(s, "0p45", 2.6)
    send(s, "0o%d" % ms, 0.05)
    send(s, "0b45@0,%d@3000;450" % target, 0.05)
    P, V, I = grab(s, ms)
    send(s, "0boff", 0.3)
    if P is None:
        return None
    base = P[0]
    d0 = (base - 45.0 + 180.0) % 360.0 - 180.0
    if abs(d0) > 3.0:
        print("  [warn] 起点 %.1f° 偏离 45°，丢弃这轮" % base)
        return None
    tgt = float(target) - 45.0
    rel = [x - base for x in P]
    land = 0
    for i in range(len(rel) - 1, 0, -1):
        if abs(rel[i] - tgt) > 2.0:
            land = i
            break
    n = len(P)
    dt = ms / 1000.0 / max(1, n)
    win = rel[land:]
    os_ = max(0.0, max(win) - tgt)
    t05 = 9999.0
    for i in range(len(win)):
        if all(abs(x - tgt) <= 0.5 for x in win[i:]):
            t05 = i * dt * 1000.0
            break
    w5 = min(len(win), max(2, int(0.5 / dt)))
    pp = max(win[:w5]) - min(win[:w5])
    tv = V[land:land + w5]
    tc = I[land:land + w5]      # ★ [WAVE] I: 才是 iq 指令；C: 是速度指令
    vpp = max(tv) - min(tv)
    ipp = max(tc) - min(tc)
    zc = sum(1 for i in range(1, len(tv)) if (tv[i - 1] > 0) != (tv[i] > 0))
    return dict(os=os_, t05=t05, pp=pp, vpp=vpp, ipp=ipp, osc=zc,
                err=rel[-1] - tgt)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None)
    ap.add_argument("--target", type=float, default=100)
    ap.add_argument("--rep", type=int, default=2)
    ap.add_argument("--env", default="8000", help="减速包络 deg/s^2")
    ap.add_argument("--no-upload", action="store_true")
    a = ap.parse_args()

    out_lines = []

    def log(txt):
        print(txt)
        out_lines.append(txt)

    if not a.no_upload:
        if not upload():
            return 1
        time.sleep(2.0)

    port = find_port(a.port)
    if not port:
        print("[X] 没找到串口。检查 USB 线，或用 --port COMx 指定。")
        return 1
    log("[串口] %s" % port)

    s = open_port(port)
    wait_ready(s)
    send(s, "0e%s" % a.env, 0.3)

    levels = [0, 115, 344, 688]
    log("")
    log("目标 %.0f°  包络 %s°/s^2  每档 %d 次" % (a.target, a.env, a.rep))
    log("%-12s %6s %7s %7s %8s %6s %5s %7s" %
        ("成形器", "os°", "t05ms", "pp°", "vpp°/s", "ipp A", "osc", "err°"))
    log("-" * 66)
    best = None
    for lv in levels:
        send(s, "0e%s,s%d" % (a.env, lv), 0.4)
        rows = []
        for _ in range(a.rep):
            r = trial(s, a.target)
            if r:
                rows.append(r)
        if not rows:
            log("%-12s  no data" % ("s%d" % lv))
            continue
        f = lambda k: statistics.mean(x[k] for x in rows)
        tag = "OFF(基线)" if lv == 0 else "s%d" % lv
        log("%-12s %6.2f %7.0f %7.2f %8.0f %6.3f %5.1f %7.2f" %
            (tag, f("os"), f("t05"), f("pp"), f("vpp"), f("ipp"), f("osc"), f("err")))
        score = f("osc") * 2.0 + f("pp") * 1.0 + f("t05") / 400.0
        if best is None or score < best[0]:
            best = (score, lv, f("osc"), f("pp"), f("t05"))

    s.close()
    if best:
        base = None
        log("-" * 66)
        log("推荐: %s  →  osc=%.1f 次, pp=%.2f°, t05=%.0fms"
            % ("OFF" if best[1] == 0 else "s%d" % best[1], best[2], best[3], best[4]))
        log("应用: 0e%s,s%d" % (a.env, best[1]))

    run_dir = os.path.join(ROOT, "runs")
    os.makedirs(run_dir, exist_ok=True)
    fn = os.path.join(run_dir, "settle_ab_%s.txt" % time.strftime("%m%d_%H%M%S"))
    with open(fn, "w", encoding="utf-8") as f:
        f.write("\n".join(out_lines))
    print("\n结果已存: %s" % fn)
    return 0


if __name__ == "__main__":
    sys.exit(main())
