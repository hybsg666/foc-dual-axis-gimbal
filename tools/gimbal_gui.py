# -*- coding: utf-8 -*-
"""
=====================================================================
 DengFOC 双轴云台指向控制上位机（Python + Tkinter）
=====================================================================
 用途 : 通过串口控制两个 FOC 电机组成的指向云台（带动激光笔等轻载）
 协议 : 115200, 8N1 —— 与固件 lib/MYFOC/foc_serial.cpp 严格对应

 上位机自动生成的命令（也可在底部"手动命令"框直接发）:
   q<m>,<位置rad>,<速度rad/s>;...   轨迹队列：清空重排，双轴切轨迹模式执行
   0e / 1e                          清空该电机队列并急停
   0r / 1r                          清除该电机故障
   p<m>,<rad> / v<m>,<rad/s> / i<m>,<A>    位置/速度/电流模式

 遥测格式（一行 14 列，逗号分隔，固件 foc_control_print 输出）:
   列 1~7  = 电机0 : 模式,目标,实际位置(rad),实际速度(rad/s),实际电流(A),电流指令(A),故障码
   列 8~14 = 电机1 : 同上
   模式码 0=电流 1=速度 2=位置 3=轨迹

 依赖 : pip install pyserial
 运行 : python gimbal_gui.py
=====================================================================
"""

import math
import queue
import threading

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None

import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox

DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi
MODE_NAMES = {0: "电流", 1: "速度", 2: "位置", 3: "轨迹"}

# 默认运动速度（rad/s）。2.0 ≈ 115°/s，指向云台够用；觉得快就调小。
DEFAULT_SPEED = 2.0


def wrap_deg(d):
    """把任意角度折回 [-180, 180)，用于直观显示"当前指向哪个角"。

    固件上报的是累积角度（unwrapped，转一圈 +360° 一直累加），
    直接显示会变成 11518° 这种看不懂的数；折回后才是真实的指向角。
    """
    d = d % 360.0
    if d >= 180.0:
        d -= 360.0
    return d


def target_text(m):
    """目标值的显示：随模式正确换算单位，值里带上单位，避免误解。"""
    mode = m["mode"]
    t = m["target"]
    if mode == 0:            # 电流模式：目标单位 A
        return f"{t:.3f} A"
    if mode == 1:            # 速度模式：目标单位 rad/s → °/s
        return f"{t * RAD2DEG:.1f} °/s"
    return f"{t * RAD2DEG:.1f} °"   # 位置/轨迹模式：目标单位 rad → °


# ------------------------------------------------------------------
# 遥测解析：一行 14 列 -> (m0, m1) 两个字典；非遥测行返回 None
# ------------------------------------------------------------------
def parse_telemetry(line):
    parts = line.split(",")
    if len(parts) != 14:
        return None
    try:
        m0 = {"mode": int(parts[0]), "target": float(parts[1]),
              "pos": float(parts[2]), "vel": float(parts[3]),
              "iq": float(parts[4]), "cmd": float(parts[5]),
              "fault": int(parts[6])}
        m1 = {"mode": int(parts[7]), "target": float(parts[8]),
              "pos": float(parts[9]), "vel": float(parts[10]),
              "iq": float(parts[11]), "cmd": float(parts[12]),
              "fault": int(parts[13])}
        return m0, m1
    except (ValueError, IndexError):
        return None


# ------------------------------------------------------------------
# 后台串口读写线程
# ------------------------------------------------------------------
class SerialWorker:
    def __init__(self, rx_queue):
        self.rx_queue = rx_queue
        self.ser = None
        self._lock = threading.Lock()
        self._running = False
        self._thread = None

    def connect(self, port, baud=115200):
        if serial is None:
            raise RuntimeError("未安装 pyserial，请先执行: pip install pyserial")
        self.disconnect()
        self.ser = serial.Serial(port, baud, timeout=0.5)
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def disconnect(self):
        self._running = False
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

    def is_open(self):
        return self.ser is not None and self.ser.is_open

    def send(self, text):
        if not self.is_open():
            return False
        with self._lock:
            try:
                self.ser.write((text + "\n").encode("utf-8"))
                return True
            except Exception:
                return False

    def _read_loop(self):
        buf = b""
        while self._running:
            try:
                data = self.ser.read(256)
            except Exception:
                break
            if not data:
                continue
            buf += data
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                line = line.strip()
                if line:
                    try:
                        self.rx_queue.put(line.decode("utf-8", errors="replace"))
                    except Exception:
                        pass


# ------------------------------------------------------------------
# 主界面
# ------------------------------------------------------------------
class GimbalApp:
    def __init__(self, root):
        self.root = root
        self.root.title("DengFOC 双轴云台指向控制")
        self.root.geometry("900x640")
        self.root.minsize(820, 600)

        self.rx_queue = queue.Queue()
        self.worker = SerialWorker(self.rx_queue)
        self.last = [None, None]          # 最近一帧遥测 m0 / m1

        self._build_ui()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.after(60, self._poll_rx)

    # ---------------- UI 搭建 ----------------
    def _build_ui(self):
        pad = {"padx": 8, "pady": 4}

        # 顶部安全提示
        tip = tk.Label(
            self.root,
            text="⚠ 安全提示：启动前确认激光/负载不对准人眼与反光面。电机0=方位角(Yaw)，电机1=俯仰角(Pitch)。",
            fg="#b00020", anchor="w", font=("Microsoft YaHei UI", 10, "bold"))
        tip.pack(fill="x", padx=10, pady=(8, 2))

        # ---- 串口连接区 ----
        conn = ttk.LabelFrame(self.root, text=" 串口连接 ")
        conn.pack(fill="x", padx=10, pady=6)

        ttk.Label(conn, text="串口:").grid(row=0, column=0, sticky="e", **pad)
        self.port_var = tk.StringVar()
        self.port_cb = ttk.Combobox(conn, textvariable=self.port_var, width=16)
        self.port_cb.grid(row=0, column=1, sticky="w", **pad)

        ttk.Label(conn, text="波特率:").grid(row=0, column=2, sticky="e", **pad)
        self.baud_var = tk.StringVar(value="115200")
        self.baud_cb = ttk.Combobox(conn, textvariable=self.baud_var, width=10,
                                    values=["115200", "460800"], state="readonly")
        self.baud_cb.grid(row=0, column=3, sticky="w", **pad)

        self.scan_btn = ttk.Button(conn, text="刷新串口", command=self._scan_ports)
        self.scan_btn.grid(row=0, column=4, **pad)

        self.conn_btn = ttk.Button(conn, text="连接", command=self._toggle_connect)
        self.conn_btn.grid(row=0, column=5, **pad)

        self.status_lbl = tk.Label(conn, text="● 未连接", fg="#888888",
                                   font=("Microsoft YaHei UI", 10, "bold"))
        self.status_lbl.grid(row=0, column=6, **pad)

        # ---- 指向控制区 ----
        ctrl = ttk.LabelFrame(self.root, text=" 指向控制（角度单位：度，程序自动换算成弧度） ")
        ctrl.pack(fill="x", padx=10, pady=6)

        # 电机0 = Yaw
        ttk.Label(ctrl, text="电机0 方位角 Yaw (°)").grid(row=0, column=0, sticky="e", **pad)
        self.yaw_var = tk.StringVar(value="0")
        ttk.Entry(ctrl, textvariable=self.yaw_var, width=12).grid(row=0, column=1, sticky="w", **pad)

        # 电机1 = Pitch
        ttk.Label(ctrl, text="电机1 俯仰角 Pitch (°)").grid(row=0, column=2, sticky="e", **pad)
        self.pitch_var = tk.StringVar(value="0")
        ttk.Entry(ctrl, textvariable=self.pitch_var, width=12).grid(row=0, column=3, sticky="w", **pad)

        ttk.Label(ctrl, text="速度 (rad/s)").grid(row=0, column=4, sticky="e", **pad)
        self.speed_var = tk.StringVar(value=str(DEFAULT_SPEED))
        ttk.Entry(ctrl, textvariable=self.speed_var, width=8).grid(row=0, column=5, sticky="w", **pad)

        ttk.Button(ctrl, text="▶ 指向", command=self._go).grid(row=0, column=6, **pad)

        # 微调按钮行
        row2 = ttk.Frame(ctrl)
        row2.grid(row=1, column=0, columnspan=8, sticky="w", padx=8, pady=2)
        ttk.Label(row2, text="微调:").pack(side="left")
        ttk.Button(row2, text="Yaw -5°", width=8, command=lambda: self._nudge(0, -5)).pack(side="left", padx=3)
        ttk.Button(row2, text="Yaw -1°", width=8, command=lambda: self._nudge(0, -1)).pack(side="left", padx=3)
        ttk.Button(row2, text="Yaw +1°", width=8, command=lambda: self._nudge(0, 1)).pack(side="left", padx=3)
        ttk.Button(row2, text="Yaw +5°", width=8, command=lambda: self._nudge(0, 5)).pack(side="left", padx=3)
        ttk.Separator(row2, orient="vertical").pack(side="left", fill="y", padx=10)
        ttk.Button(row2, text="Pitch -5°", width=9, command=lambda: self._nudge(1, -5)).pack(side="left", padx=3)
        ttk.Button(row2, text="Pitch -1°", width=9, command=lambda: self._nudge(1, -1)).pack(side="left", padx=3)
        ttk.Button(row2, text="Pitch +1°", width=9, command=lambda: self._nudge(1, 1)).pack(side="left", padx=3)
        ttk.Button(row2, text="Pitch +5°", width=9, command=lambda: self._nudge(1, 5)).pack(side="left", padx=3)

        # 通用按钮行
        row3 = ttk.Frame(ctrl)
        row3.grid(row=2, column=0, columnspan=8, sticky="w", padx=8, pady=4)
        ttk.Button(row3, text="读取当前角", command=self._read_pos_as_target).pack(side="left", padx=3)
        ttk.Button(row3, text="回零", command=self._home).pack(side="left", padx=3)
        ttk.Button(row3, text="■ 急停", command=self._estop).pack(side="left", padx=3)
        ttk.Button(row3, text="清故障", command=self._clear_fault).pack(side="left", padx=3)

        # ---- 实时状态区 ----
        state = ttk.LabelFrame(self.root, text=" 实时状态（固件遥测解析） ")
        state.pack(fill="both", expand=True, padx=10, pady=6)

        cols = ("项目", "电机0 (Yaw)", "电机1 (Pitch)")
        self.tree = ttk.Treeview(state, columns=cols, show="headings", height=8)
        for c in cols:
            self.tree.heading(c, text=c)
            self.tree.column(c, width=200, anchor="center")
        self.tree.pack(fill="both", expand=True, padx=6, pady=6)

        rows = ["模式", "目标值", "实际角(°)", "累计角(°)",
                "速度(°/s)", "电流(A)", "电流指令(A)", "故障码"]
        for r in rows:
            self.tree.insert("", "end", values=(r, "-", "-"))

        # ---- 日志 + 手动命令区 ----
        bottom = ttk.LabelFrame(self.root, text=" 日志 & 手动命令 ")
        bottom.pack(fill="both", expand=True, padx=10, pady=6)

        cmd = ttk.Frame(bottom)
        cmd.pack(fill="x", padx=6, pady=4)
        ttk.Label(cmd, text="手动命令:").pack(side="left")
        self.cmd_var = tk.StringVar()
        cmd_entry = ttk.Entry(cmd, textvariable=self.cmd_var)
        cmd_entry.pack(side="left", fill="x", expand=True, padx=6)
        cmd_entry.bind("<Return>", lambda e: self._send_manual())
        ttk.Button(cmd, text="发送", command=self._send_manual).pack(side="left")

        self.log = scrolledtext.ScrolledText(bottom, height=8, state="disabled",
                                             font=("Consolas", 9))
        self.log.pack(fill="both", expand=True, padx=6, pady=(0, 6))

        self._scan_ports()
        self._log("就绪。选择串口后点『连接』。")

    # ---------------- 串口操作 ----------------
    def _scan_ports(self):
        if list_ports is None:
            return
        ports = [p.device for p in list_ports.comports()]
        self.port_cb["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def _toggle_connect(self):
        if self.worker.is_open():
            self.worker.disconnect()
            self.status_lbl.config(text="● 未连接", fg="#888888")
            self.conn_btn.config(text="连接")
            self._log("已断开串口。")
        else:
            port = self.port_var.get().strip()
            if not port:
                messagebox.showwarning("提示", "请先选择串口（可点『刷新串口』）。")
                return
            try:
                self.worker.connect(port, int(self.baud_var.get()))
            except Exception as e:
                messagebox.showerror("连接失败", str(e))
                return
            self.status_lbl.config(text="● 已连接", fg="#1a7f37")
            self.conn_btn.config(text="断开")
            self._log(f"已连接 {port} @ {self.baud_var.get()}。")

    def _send(self, text):
        if not self.worker.is_open():
            self._log("✗ 未连接串口，命令未发送: " + text)
            return False
        if self.worker.send(text):
            self._log(">> " + text)
            return True
        self._log("✗ 发送失败: " + text)
        return False

    # ---------------- 控制动作 ----------------
    def _float(self, var, default=0.0):
        try:
            return float(var.get().strip())
        except ValueError:
            return default

    def _current_targets(self):
        return self._float(self.yaw_var), self._float(self.pitch_var)

    def _go(self):
        yaw_deg, pitch_deg = self._current_targets()
        spd = self._float(self.speed_var, DEFAULT_SPEED)
        yaw_rad = yaw_deg * DEG2RAD
        pitch_rad = pitch_deg * DEG2RAD
        cmd = f"q0,{yaw_rad:.4f},{spd:.3f};1,{pitch_rad:.4f},{spd:.3f}"
        self._send(cmd)

    def _nudge(self, motor, step_deg):
        """在目标角基础上微调。motor: 0=yaw, 1=pitch"""
        var = self.yaw_var if motor == 0 else self.pitch_var
        cur = self._float(var)
        var.set(f"{cur + step_deg:.1f}")
        self._go()

    def _home(self):
        self.yaw_var.set("0")
        self.pitch_var.set("0")
        self._go()

    def _estop(self):
        self._send("0e")
        self._send("1e")

    def _clear_fault(self):
        self._send("0r")
        self._send("1r")

    def _read_pos_as_target(self):
        """把最近一帧遥测的实际角度填进目标框（用于停住后微调）。"""
        if self.last[0] is None or self.last[1] is None:
            messagebox.showinfo("提示", "还没有收到遥测，请确认串口已连接且固件在打印。")
            return
        self.yaw_var.set(f"{self.last[0]['pos'] * RAD2DEG:.1f}")
        self.pitch_var.set(f"{self.last[1]['pos'] * RAD2DEG:.1f}")
        self._log("已把当前实际角度写入目标框。")

    def _send_manual(self):
        text = self.cmd_var.get().strip()
        if text:
            self._send(text)
            self.cmd_var.set("")

    # ---------------- 接收处理 ----------------
    def _poll_rx(self):
        try:
            while True:
                line = self.rx_queue.get_nowait()
                self._handle_line(line)
        except queue.Empty:
            pass
        self.root.after(60, self._poll_rx)

    def _handle_line(self, line):
        t = parse_telemetry(line)
        if t is not None:
            self.last[0], self.last[1] = t
            self._update_state(t)
        else:
            self._log(line)

    def _update_state(self, t):
        m0, m1 = t
        kids = self.tree.get_children()
        self.tree.item(kids[0],
                       values=("模式", MODE_NAMES.get(m0["mode"], "?"), MODE_NAMES.get(m1["mode"], "?")))
        self.tree.item(kids[1],
                       values=("目标值", target_text(m0), target_text(m1)))
        self.tree.item(kids[2],
                       values=("实际角(°)", f"{wrap_deg(m0['pos'] * RAD2DEG):.1f}",
                               f"{wrap_deg(m1['pos'] * RAD2DEG):.1f}"))
        self.tree.item(kids[3],
                       values=("累计角(°)", f"{m0['pos'] * RAD2DEG:.1f}", f"{m1['pos'] * RAD2DEG:.1f}"))
        self.tree.item(kids[4],
                       values=("速度(°/s)", f"{m0['vel'] * RAD2DEG:.0f}", f"{m1['vel'] * RAD2DEG:.0f}"))
        self.tree.item(kids[5],
                       values=("电流(A)", f"{m0['iq']:.3f}", f"{m1['iq']:.3f}"))
        self.tree.item(kids[6],
                       values=("电流指令(A)", f"{m0['cmd']:.3f}", f"{m1['cmd']:.3f}"))
        self.tree.item(kids[7],
                       values=("故障码", str(m0["fault"]), str(m1["fault"])))

    def _log(self, text):
        self.log.config(state="normal")
        self.log.insert("end", text + "\n")
        self.log.see("end")
        self.log.config(state="disabled")

    def _on_close(self):
        self.worker.disconnect()
        self.root.destroy()


def main():
    root = tk.Tk()
    GimbalApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
