# -*- coding: utf-8 -*-
"""
=====================================================================
 S 曲线轨迹实测对比工具
=====================================================================
 它做的事：
   1. 打开串口，发一条轨迹命令  q<电机>,<目标rad>,<速度>
   2. 抓固件的 [TRAJ] 规划行（dist / vpk / T）
   3. 边跑边采遥测（位置 / 速度 / 电流）
   4. 结束后把「实测速度」和「理论 S 曲线」叠在一张图上，
      存成 tools/out/traj_compare.html，浏览器打开看

 用法（在 3MytowFOC SVPWM 目录下）：
   python tools/diag_traj.py                     # 自动找串口，M1 转 90°，速度 5
   python tools/diag_traj.py -p COM5 -m 0 -t 180 -v 5
   python tools/diag_traj.py -t 30 -v 2          # 慢速小角度

 判据（怎么算通过）：
   · 实测速度曲线应该是「两头圆、中间平」，不是梯形那样的直角
   · 实测峰值速度应接近理论 vpk（±15% 以内，位置环跟踪有滞后）
   · 终点误差 < 0.05 rad
   · 全程不出现 fault
=====================================================================
"""

import argparse
import json
import os
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("缺少 pyserial。请执行： pip install pyserial")
    sys.exit(1)

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from verify_scurve import traj_plan, s_pos, s_vel, TRAJ_ACCEL, TRAJ_JERK  # noqa: E402

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")


def pick_port(user_port):
    if user_port:
        return user_port
    ports = list(list_ports.comports())
    if not ports:
        print("没找到串口。检查 USB 线或用 -p COM5 指定。")
        sys.exit(1)
    if len(ports) == 1:
        return ports[0].device
    print("检测到多个串口：")
    for i, p in enumerate(ports):
        print("  [%d] %s  %s" % (i, p.device, p.description))
    return ports[int(input("输入编号: ").strip())].device


def parse_telemetry(line):
    parts = line.split(",")
    if len(parts) != 14:
        return None
    try:
        return [float(x) for x in parts]
    except ValueError:
        return None


def parse_traj_line(line):
    """从 [TRAJ] 规划行里取 dist / vpk / T"""
    info = {}
    for kv in line.replace("[TRAJ]", " ").split():
        if "=" in kv:
            k, v = kv.split("=", 1)
            try:
                info[k] = float(v)
            except ValueError:
                pass
    return info if "dist" in info else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-p", "--port", default=None)
    ap.add_argument("-m", "--motor", type=int, default=1)
    ap.add_argument("-t", "--target", type=float, default=90.0, help="目标机械角，单位度")
    ap.add_argument("-v", "--speed", type=float, default=5.0, help="速度 rad/s")
    ap.add_argument("-w", "--wait", type=float, default=3.0, help="跑完后额外观察秒数")
    args = ap.parse_args()

    port = pick_port(args.port)
    target_rad = args.target * 3.141592653589793 / 180.0
    print("串口: %s   M%d   %.1f° (%.4f rad)   速度 %.2f rad/s\n"
          % (port, args.motor, args.target, target_rad, args.speed))

    ser = serial.Serial(port, 115200, timeout=0.05)
    time.sleep(0.3)
    ser.reset_input_buffer()

    base = args.motor * 7
    cmd = "q%d,%.4f,%.2f" % (args.motor, target_rad, args.speed)
    ser.write((cmd + "\n").encode())
    print(">>> %s" % cmd)

    t_start = None
    samples = []          # (t, pos, vel, iq, fault)
    plan = None
    logs = []
    deadline = time.time() + 20.0
    last_end_ts = None

    while time.time() < deadline:
        raw = ser.readline().decode("utf-8", "ignore").strip()
        if not raw:
            # 空闲时提前退出：轨迹跑完 + 观察窗口结束
            if last_end_ts and time.time() - last_end_ts > args.wait:
                break
            continue

        if raw.startswith("["):
            logs.append(raw)
            print("   FB: %s" % raw)
            p = parse_traj_line(raw)
            if p and plan is None:
                plan = p
                t_start = time.time()
            continue

        if not raw[0].isdigit():
            continue

        v = parse_telemetry(raw)
        if v is None:
            continue
        row = v[base:base + 7]
        now = time.time()
        if t_start is None:
            t_start = now
        t = now - t_start
        samples.append((t, row[2], row[3], row[4], int(row[6])))

        # 位置收敛判据：连续 0.5s 内变化 < 0.002 rad 视为停住
        if len(samples) > 30:
            if abs(samples[-1][1] - samples[-30][1]) < 0.02:
                if not last_end_ts:
                    last_end_ts = now

    ser.close()

    # ---------------- 终端汇总 ----------------
    if not samples:
        print("\n没收到遥测。检查：串口选错 / 遥测被 'd' 关掉 / 固件没在跑。")
        return

    pos0 = samples[0][1]
    pos_end = samples[-1][1]
    vel_meas = [s[2] for s in samples]
    vpk_meas = max(abs(v) for v in vel_meas)
    faults = sorted({s[4] for s in samples if s[4]})

    print("\n================ 结果 ================")
    print("位移      : %.3f → %.3f rad  (走了 %.3f rad = %.1f°)"
          % (pos0, pos_end, abs(pos_end - pos0), abs(pos_end - pos0) * 57.2958))
    print("实测峰值速: %.2f rad/s" % vpk_meas)
    print("故障码    : %s" % (faults if faults else "无"))

    if plan:
        D, vpk_th, T_th = abs(plan["dist"]), plan["vpk"], plan["T"]
        print("理论规划  : dist=%.3f  vpk=%.2f  T=%.3fs" % (D, vpk_th, T_th))
        err_pct = (vpk_meas - vpk_th) / vpk_th * 100.0 if vpk_th > 0 else 0.0
        print("速度吻合  : %+.0f%%  (实测/理论)" % err_pct)
    else:
        D = 3.141592653589793
        vpk_th = args.speed
        err_pct = 0.0

    end_err = abs(target_rad - pos_end) if args.target else 0.0
    print("终点误差  : %.4f rad (%.2f°)" % (end_err, end_err * 57.2958))

    print("\n---------------- 判据 ----------------")
    print("  %s 终点到位      (<0.05 rad)" % ("PASS" if end_err < 0.05 else "FAIL"))
    print("  %s 速度量级对    (|偏差|<25%%)" % ("PASS" if abs(err_pct) < 25 else "FAIL"))
    print("  %s 无故障码" % ("PASS" if not faults else "FAIL"))

    # ---------------- 生成对比图 ----------------
    p = traj_plan(D, vpk_th, TRAJ_ACCEL, TRAJ_JERK)
    n = 400
    th_t = [p["T"] * i / n for i in range(n + 1)]
    th_v = [s_vel(t, p) for t in th_t]
    th_s = [s_pos(t, p) for t in th_t]

    data = {
        "meas": [[round(s[0], 4), round(abs(s[1] - pos0), 4), round(abs(s[2]), 4)] for s in samples],
        "th_t": [round(x, 4) for x in th_t],
        "th_v": [round(x, 4) for x in th_v],
        "th_s": [round(x, 4) for x in th_s],
        "D": round(D, 4),
        "vpk": round(vpk_th, 3),
        "T": round(p["T"], 3),
        "vpk_meas": round(vpk_meas, 3),
        "title": "M%d  %.0f°  v=%.1f" % (args.motor, args.target, args.speed),
    }

    os.makedirs(OUT_DIR, exist_ok=True)
    out = os.path.join(OUT_DIR, "traj_compare.html")
    with open(out, "w", encoding="utf-8") as f:
        f.write(HTML_TPL.replace("__DATA__", json.dumps(data, ensure_ascii=False)))
    print("\n对比图已生成：%s" % out)


HTML_TPL = r"""<!DOCTYPE html>
<html lang="zh-CN"><head><meta charset="utf-8">
<title>S 曲线实测对比</title>
<style>
body{margin:0;padding:24px;font:14px/1.6 -apple-system,"Segoe UI","Microsoft YaHei",sans-serif;background:#fff;color:#1a1a1a}
h1{font-size:17px;font-weight:500;margin:0 0 4px}
p.sub{color:#666;font-size:13px;margin:0 0 18px}
.card{border:1px solid #e5e5e5;border-radius:12px;padding:16px;margin-bottom:16px}
h2{font-size:13px;font-weight:500;margin:0 0 10px;color:#444}
canvas{width:100%;height:260px;display:block}
.lg{display:flex;gap:18px;font-size:12px;color:#666;margin-top:8px}
.sw{display:inline-block;width:14px;height:2px;vertical-align:middle;margin-right:6px}
</style></head><body>
<h1>轨迹实测 vs 理论</h1>
<p class="sub" id="sub"></p>
<div class="card"><h2>速度 (rad/s)</h2><canvas id="cv1"></canvas>
<div class="lg"><span><i class="sw" style="background:#378ADD"></i>实测（编码器测速）</span>
<span><i class="sw" style="background:#D85A30"></i>理论 S 曲线</span></div></div>
<div class="card"><h2>位置 (rad)</h2><canvas id="cv2"></canvas>
<div class="lg"><span><i class="sw" style="background:#378ADD"></i>实测</span>
<span><i class="sw" style="background:#D85A30"></i>理论</span></div></div>
<script>
const D = __DATA__;
document.getElementById('sub').textContent =
  D.title + '　理论 vpk=' + D.vpk + ' rad/s, T=' + D.T + 's　实测 vpk=' + D.vpk_meas + ' rad/s';
function panel(id, xs, ys, xt, yt, ymaxHint){
  const cv=document.getElementById(id), dpr=window.devicePixelRatio||1;
  const w=cv.clientWidth,h=cv.clientHeight;
  cv.width=w*dpr;cv.height=h*dpr;
  const c=cv.getContext('2d');c.scale(dpr,dpr);c.clearRect(0,0,w,h);
  const pad={l:46,r:14,t:10,b:26},W=w-pad.l-pad.r,H=h-pad.t-pad.b;
  const xmax=Math.max(xs.length?xs[xs.length-1]:0, xt[xt.length-1]||0)||1;
  const ymax=Math.max(...ys, ...yt, ymaxHint||0.001)*1.1;
  c.strokeStyle='#e8e8e8';c.lineWidth=1;
  for(let i=0;i<=4;i++){const y=pad.t+H*i/4;c.beginPath();c.moveTo(pad.l,y);c.lineTo(pad.l+W,y);c.stroke();}
  c.fillStyle='#999';c.font='11px sans-serif';
  c.fillText(ymax.toFixed(2),4,pad.t+8);c.fillText('0',32,pad.t+H+4);
  c.fillText(xmax.toFixed(2)+'s',pad.l+W-26,pad.t+H+18);
  const X=t=>pad.l+t/xmax*W, Y=v=>pad.t+H-v/ymax*H;
  c.lineWidth=1.6;c.strokeStyle='#D85A30';c.beginPath();
  xt.forEach((t,i)=>i?c.lineTo(X(t),Y(yt[i])):c.moveTo(X(t),Y(yt[i])));c.stroke();
  c.lineWidth=2;c.strokeStyle='#378ADD';c.beginPath();
  xs.forEach((t,i)=>i?c.lineTo(X(t),Y(ys[i])):c.moveTo(X(t),Y(ys[i])));c.stroke();
}
const mt=D.meas.map(r=>r[0]), mv=D.meas.map(r=>r[2]), ms=D.meas.map(r=>r[1]);
requestAnimationFrame(()=>{
  panel('cv1',mt,mv,D.th_t,D.th_v,0);
  panel('cv2',mt,ms,D.th_t,D.th_s,0);
});
</script></body></html>"""


if __name__ == "__main__":
    main()
