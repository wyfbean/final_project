#!/usr/bin/env python3
"""Desktop visualizer for the STM32 maze SLAM text protocol.

Usage examples:
  python tools/slam_visualizer.py --port COM7
  python tools/slam_visualizer.py --replay logs/session.txt

Install pyserial for live serial use:
  python -m pip install pyserial
"""

from __future__ import annotations

import argparse
import math
import queue
import re
import threading
import time
import tkinter as tk
from dataclasses import dataclass, field
from tkinter import filedialog, messagebox, ttk
from typing import Dict, Iterable, List, Optional, Tuple

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover - live serial is optional for replay mode.
    serial = None
    list_ports = None


DEFAULT_BAUD = 921600
DEFAULT_PORT = "COM6"
GRID_W = 56
GRID_H = 56
CELL_MM = 175
CANVAS_SIZE = 720
LOG_LIMIT = 300
LOG_SUPPRESSED_PREFIXES = ("MAP ROW ",)
BLUETOOTH_KEYWORDS = ("bluetooth", "standard serial over bluetooth", "bth", "spp")

MAP_HEADER_RE = re.compile(r"MAP\s+(?P<state>\S+).*w=(?P<w>\d+)\s+h=(?P<h>\d+)\s+cell=(?P<cell>\d+)mm\s+rev=(?P<rev>\d+)")
MAP_ROW_RE = re.compile(r"MAP ROW y=(?P<y>\d+)\s+rev=(?P<rev>\d+)\s+data=(?P<data>[.#?]+)")
MAP_STAT_RE = re.compile(r"MAP STAT\s+(?P<body>.*)")
POSE_RE = re.compile(r"POSE\s+(-?\d+),(-?\d+),(-?\d+)")
PATH_BEGIN_RE = re.compile(r"PATH BEGIN seq=(?P<seq>\d+)\s+rev=(?P<rev>\d+)\s+len=(?P<len>\d+)\s+target=(?P<x>\d+),(?P<y>\d+)")
PATH_CHUNK_RE = re.compile(r"PATH CHUNK seq=(?P<seq>\d+)\s+idx=(?P<idx>\d+)\s+data=(?P<data>.*)")
PATH_END_RE = re.compile(r"PATH END seq=(?P<seq>\d+)")
SLAM_HB_RE = re.compile(
    r"SLAM HB state=(?P<state>\S+)\s+seq=(?P<seq>\d+)\s+"
    r"(?:cell=(?P<cell_x>\d+),(?P<cell_y>\d+)\s+)?"
    r"target=(?P<x>\d+),(?P<y>\d+)\s+path_i=(?P<path_i>\d+)\s+"
    r"path_len=(?P<path_len>\d+)\s+front=(?P<front>\d+)\s+rev=(?P<rev>\d+)"
)
SLAM_STATE_RE = re.compile(r"SLAM state=(?P<state>\S+)\s+reason=(?P<reason>\S+)\s+seq=(?P<seq>\d+).*")


def list_serial_port_labels() -> List[str]:
    if list_ports is None:
        return []

    labels: List[str] = []
    for port in list_ports.comports():
        details = " ".join(
            item for item in (port.description, port.manufacturer, port.hwid) if item
        )
        labels.append(f"{port.device}  {details}".strip())
    return labels


def extract_port_name(label: str) -> str:
    return label.strip().split()[0] if label.strip() else ""


def pick_bluetooth_port_label(labels: List[str]) -> Optional[str]:
    for label in labels:
        lower = label.lower()
        if any(keyword in lower for keyword in BLUETOOTH_KEYWORDS):
            return label
    return labels[0] if len(labels) == 1 else None


@dataclass
class Pose:
    x_mm: int = 0
    y_mm: int = 0
    heading_cdeg: int = 0
    valid: bool = False


@dataclass
class SlamStatus:
    state: str = "IDLE"
    reason: str = ""
    seq: int = 0
    cell: Optional[Tuple[int, int]] = None
    target: Optional[Tuple[int, int]] = None
    path_i: int = 0
    path_len: int = 0
    front_mm: int = 0
    revision: int = 0


@dataclass
class PathBuild:
    seq: int
    expected_len: int
    points: Dict[int, Tuple[int, int]] = field(default_factory=dict)


@dataclass
class Model:
    width: int = GRID_W
    height: int = GRID_H
    cell_mm: int = CELL_MM
    revision: int = 0
    grid: List[List[str]] = field(default_factory=lambda: [["?" for _ in range(GRID_W)] for _ in range(GRID_H)])
    pose: Pose = field(default_factory=Pose)
    path: List[Tuple[int, int]] = field(default_factory=list)
    path_seq: int = 0
    pending_path: Optional[PathBuild] = None
    slam: SlamStatus = field(default_factory=SlamStatus)
    map_state: str = "IDLE"
    connected: bool = False
    source: str = "offline"
    last_line: str = ""
    raw_log: List[str] = field(default_factory=list)
    rows_received: int = 0
    free_cells: int = 0
    occupied_cells: int = 0
    unknown_cells: int = GRID_W * GRID_H
    tx_drops: int = 0
    inserted_points: int = 0
    lidar_quality_min: int = 5
    lidar_distance_bias_mm: int = 0

    def reset_grid(self, width: int = GRID_W, height: int = GRID_H, cell_mm: int = CELL_MM) -> None:
        self.width = width
        self.height = height
        self.cell_mm = cell_mm
        self.grid = [["?" for _ in range(width)] for _ in range(height)]
        self.path.clear()
        self.pending_path = None
        self.rows_received = 0


class ProtocolParser:
    def __init__(self, model: Model) -> None:
        self.model = model

    def parse_line(self, line: str) -> None:
        line = line.strip()
        if not line:
            return

        self.model.last_line = line
        if not line.startswith(LOG_SUPPRESSED_PREFIXES):
            self.model.raw_log.append(line)
            if len(self.model.raw_log) > LOG_LIMIT:
                del self.model.raw_log[: len(self.model.raw_log) - LOG_LIMIT]

        if match := MAP_HEADER_RE.match(line):
            state = match.group("state")
            self.model.map_state = state
            width = int(match.group("w"))
            height = int(match.group("h"))
            cell_mm = int(match.group("cell"))
            self.model.revision = int(match.group("rev"))
            if (
                state == "START"
                or width != self.model.width
                or height != self.model.height
                or cell_mm != self.model.cell_mm
            ):
                self.model.reset_grid(width, height, cell_mm)
            return

        if match := MAP_ROW_RE.match(line):
            y = int(match.group("y"))
            data = match.group("data")
            self.model.revision = int(match.group("rev"))
            if 0 <= y < self.model.height:
                row = list(data[: self.model.width])
                if len(row) < self.model.width:
                    row.extend("?" for _ in range(self.model.width - len(row)))
                self.model.grid[y] = row
                self.model.rows_received += 1
            return

        if match := MAP_STAT_RE.match(line):
            fields = self._parse_key_values(match.group("body"))
            self.model.revision = self._get_int(fields, "rev", self.model.revision)
            self.model.inserted_points = self._get_int(fields, "inserted", self.model.inserted_points)
            self.model.unknown_cells = self._get_int(fields, "unknown", self.model.unknown_cells)
            self.model.free_cells = self._get_int(fields, "free", self.model.free_cells)
            self.model.occupied_cells = self._get_int(fields, "occupied", self.model.occupied_cells)
            self.model.tx_drops = self._get_int(fields, "txdrop", self.model.tx_drops)
            self.model.lidar_quality_min = self._get_int(fields, "qmin", self.model.lidar_quality_min)
            self.model.lidar_distance_bias_mm = self._get_int(fields, "bias", self.model.lidar_distance_bias_mm)
            if "pose" in fields:
                try:
                    x_text, y_text, h_text = fields["pose"].split(",", 2)
                    self.model.pose = Pose(
                        x_mm=int(x_text),
                        y_mm=int(y_text),
                        heading_cdeg=int(h_text),
                        valid=True,
                    )
                except ValueError:
                    pass
            return

        if match := POSE_RE.match(line):
            self.model.pose = Pose(
                x_mm=int(match.group(1)),
                y_mm=int(match.group(2)),
                heading_cdeg=int(match.group(3)),
                valid=True,
            )
            return

        if match := PATH_BEGIN_RE.match(line):
            seq = int(match.group("seq"))
            self.model.pending_path = PathBuild(seq=seq, expected_len=int(match.group("len")))
            self.model.path_seq = seq
            self.model.slam.target = (int(match.group("x")), int(match.group("y")))
            self.model.slam.revision = int(match.group("rev"))
            return

        if match := PATH_CHUNK_RE.match(line):
            pending = self.model.pending_path
            if pending is None or pending.seq != int(match.group("seq")):
                return
            index = int(match.group("idx"))
            for offset, item in enumerate(filter(None, match.group("data").split(";"))):
                try:
                    x_text, y_text = item.split(",", 1)
                    pending.points[index + offset] = (int(x_text), int(y_text))
                except ValueError:
                    continue
            return

        if match := PATH_END_RE.match(line):
            pending = self.model.pending_path
            if pending is None or pending.seq != int(match.group("seq")):
                return
            self.model.path = [
                pending.points[i]
                for i in range(pending.expected_len)
                if i in pending.points
            ]
            self.model.pending_path = None
            return

        if match := SLAM_HB_RE.match(line):
            self.model.slam.state = match.group("state")
            self.model.slam.seq = int(match.group("seq"))
            if match.group("cell_x") is not None and match.group("cell_y") is not None:
                self.model.slam.cell = (int(match.group("cell_x")), int(match.group("cell_y")))
            self.model.slam.target = (int(match.group("x")), int(match.group("y")))
            self.model.slam.path_i = int(match.group("path_i"))
            self.model.slam.path_len = int(match.group("path_len"))
            self.model.slam.front_mm = int(match.group("front"))
            self.model.slam.revision = int(match.group("rev"))
            return

        if match := SLAM_STATE_RE.match(line):
            self.model.slam.state = match.group("state")
            self.model.slam.reason = match.group("reason")
            self.model.slam.seq = int(match.group("seq"))

    @staticmethod
    def _parse_key_values(body: str) -> Dict[str, str]:
        fields: Dict[str, str] = {}
        for item in body.split():
            if "=" not in item:
                continue
            key, value = item.split("=", 1)
            fields[key] = value
        return fields

    @staticmethod
    def _get_int(fields: Dict[str, str], key: str, default: int) -> int:
        value = fields.get(key)
        if value is None:
            return default

        try:
            return int(value)
        except ValueError:
            return default


class SerialWorker:
    def __init__(self, event_queue: queue.Queue[str]) -> None:
        self.event_queue = event_queue
        self.serial_obj = None
        self.thread: Optional[threading.Thread] = None
        self.stop_event = threading.Event()

    def connect(self, port: str, baud: int) -> None:
        if serial is None:
            raise RuntimeError("pyserial is not installed. Run: python -m pip install pyserial")

        self.disconnect()
        self.stop_event.clear()
        self.serial_obj = serial.Serial(
            port=port,
            baudrate=baud,
            timeout=0.05,
            write_timeout=1.0,
            rtscts=False,
            dsrdtr=False,
        )
        try:
            self.serial_obj.setDTR(False)
            self.serial_obj.setRTS(False)
            self.serial_obj.reset_input_buffer()
        except Exception:
            pass
        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()

    def replay(self, path: str, delay_s: float = 0.01) -> None:
        self.disconnect()
        self.stop_event.clear()
        self.thread = threading.Thread(target=self._replay_loop, args=(path, delay_s), daemon=True)
        self.thread.start()

    def disconnect(self) -> None:
        self.stop_event.set()
        if self.serial_obj is not None:
            try:
                self.serial_obj.close()
            except Exception:
                pass
        self.serial_obj = None

    def write_line(self, text: str) -> None:
        if self.serial_obj is None or not self.serial_obj.is_open:
            return
        payload = (text.strip() + "\r\n").encode("ascii", errors="ignore")
        self.serial_obj.write(payload)

    def _read_loop(self) -> None:
        while not self.stop_event.is_set() and self.serial_obj is not None:
            try:
                raw = self.serial_obj.readline()
            except Exception as exc:
                self.event_queue.put(f"ERROR {exc}")
                break
            if raw:
                self.event_queue.put(raw.decode("ascii", errors="replace").strip())

    def _replay_loop(self, path: str, delay_s: float) -> None:
        try:
            with open(path, "r", encoding="utf-8", errors="replace") as handle:
                for line in handle:
                    if self.stop_event.is_set():
                        break
                    self.event_queue.put(line.strip())
                    time.sleep(delay_s)
        except OSError as exc:
            self.event_queue.put(f"ERROR {exc}")


class SlamVisualizer(tk.Tk):
    def __init__(self, port: Optional[str], baud: int, replay_path: Optional[str]) -> None:
        super().__init__()
        self.title("STM32 SLAM / A* Visualizer")
        self.geometry("1060x820")

        self.model = Model()
        self.parser = ProtocolParser(self.model)
        self.events: queue.Queue[str] = queue.Queue()
        self.worker = SerialWorker(self.events)
        self.baud_var = tk.IntVar(value=baud)
        self.lidar_quality_var = tk.IntVar(value=5)
        self.port_var = tk.StringVar(value=port or DEFAULT_PORT)
        self.port_labels: List[str] = []
        self.status_vars = {
            "source": tk.StringVar(value="offline"),
            "map": tk.StringVar(value="MAP IDLE rev=0"),
            "pose": tk.StringVar(value="POSE --"),
            "slam": tk.StringVar(value="SLAM IDLE"),
            "path": tk.StringVar(value="PATH --"),
            "last": tk.StringVar(value=""),
        }

        self._build_ui()
        self._refresh_ports(auto_select=(port is None))
        if replay_path:
            self._start_replay(replay_path)
        elif port:
            self._connect()

        self.after(33, self._tick)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self) -> None:
        toolbar = ttk.Frame(self)
        toolbar.pack(side=tk.TOP, fill=tk.X, padx=8, pady=6)

        ttk.Label(toolbar, text="Port").pack(side=tk.LEFT)
        self.port_combo = ttk.Combobox(toolbar, textvariable=self.port_var, width=36)
        self.port_combo.pack(side=tk.LEFT, padx=(4, 8))
        ttk.Button(toolbar, text="Refresh", command=lambda: self._refresh_ports(auto_select=False)).pack(side=tk.LEFT, padx=2)
        ttk.Button(toolbar, text="Auto BT", command=self._auto_select_bluetooth).pack(side=tk.LEFT, padx=2)
        ttk.Label(toolbar, text="Baud").pack(side=tk.LEFT)
        ttk.Entry(toolbar, textvariable=self.baud_var, width=8).pack(side=tk.LEFT, padx=(4, 8))
        ttk.Button(toolbar, text="Connect", command=self._connect).pack(side=tk.LEFT, padx=2)
        ttk.Button(toolbar, text="Disconnect", command=self._disconnect).pack(side=tk.LEFT, padx=2)
        ttk.Button(toolbar, text="Replay Log", command=self._choose_replay).pack(side=tk.LEFT, padx=2)

        command_bar = ttk.Frame(self)
        command_bar.pack(side=tk.TOP, fill=tk.X, padx=8, pady=(0, 6))
        ttk.Button(command_bar, text="SLAM", command=self._start_slam).pack(side=tk.LEFT, padx=2)
        ttk.Button(command_bar, text="Start Map", command=self._start_map).pack(side=tk.LEFT, padx=2)
        ttk.Button(command_bar, text="Map + 96", command=self._start_map_auto96).pack(side=tk.LEFT, padx=2)
        ttk.Button(command_bar, text="Back", command=lambda: self._send_command("BACK")).pack(side=tk.LEFT, padx=2)
        ttk.Button(command_bar, text="0 Brake", command=lambda: self._send_command("0")).pack(side=tk.LEFT, padx=2)
        ttk.Button(command_bar, text="Reset Map North", command=lambda: self._send_command("RESET FRONT")).pack(side=tk.LEFT, padx=2)
        ttk.Button(command_bar, text="Gyro Cal", command=lambda: self._send_command("GYRO CAL")).pack(side=tk.LEFT, padx=2)
        ttk.Label(command_bar, text="Q>=").pack(side=tk.LEFT, padx=(8, 2))
        ttk.Entry(command_bar, textvariable=self.lidar_quality_var, width=4).pack(side=tk.LEFT, padx=2)
        ttk.Button(command_bar, text="Set Q", command=self._set_lidar_quality).pack(side=tk.LEFT, padx=2)
        ttk.Button(command_bar, text="94 Odom 350mm", command=lambda: self._send_command("94")).pack(side=tk.LEFT, padx=2)
        ttk.Button(command_bar, text="End Encoder", command=lambda: self._send_command("END ENCODER")).pack(side=tk.LEFT, padx=2)
        ttk.Button(command_bar, text="MPU State", command=lambda: self._send_command("MPU STATE")).pack(side=tk.LEFT, padx=2)
        ttk.Button(command_bar, text="DIR", command=lambda: self._send_command("DIR")).pack(side=tk.LEFT, padx=2)
        ttk.Button(command_bar, text="Stop SLAM", command=lambda: self._send_command("SLAM OFF")).pack(side=tk.LEFT, padx=2)
        ttk.Button(command_bar, text="Show Map", command=lambda: self._send_command("SHOW MAP")).pack(side=tk.LEFT, padx=2)

        main = ttk.Frame(self)
        main.pack(fill=tk.BOTH, expand=True, padx=8, pady=(0, 8))

        self.canvas = tk.Canvas(main, width=CANVAS_SIZE, height=CANVAS_SIZE, bg="#20242a", highlightthickness=0)
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=False)

        side = ttk.Frame(main, width=300)
        side.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(10, 0))

        for label, var in self.status_vars.items():
            ttk.Label(side, text=label.upper()).pack(anchor=tk.W, pady=(0 if label == "source" else 10, 0))
            ttk.Label(side, textvariable=var, wraplength=290).pack(anchor=tk.W)

        ttk.Label(side, text="LOG").pack(anchor=tk.W, pady=(12, 0))
        self.log_box = tk.Text(side, height=24, width=44, state=tk.DISABLED)
        self.log_box.pack(fill=tk.BOTH, expand=True)

    def _refresh_ports(self, auto_select: bool) -> None:
        self.port_labels = list_serial_port_labels()
        self.port_combo.configure(values=self.port_labels)
        if auto_select:
            selected = pick_bluetooth_port_label(self.port_labels)
            if selected is not None:
                self.port_var.set(selected)

    def _auto_select_bluetooth(self) -> None:
        self._refresh_ports(auto_select=False)
        selected = pick_bluetooth_port_label(self.port_labels)
        if selected is None:
            messagebox.showwarning("Bluetooth port", "No Bluetooth-looking COM port found. Pair the module in Windows first.")
            return
        self.port_var.set(selected)

    def _connect(self) -> None:
        port = extract_port_name(self.port_var.get())
        if not port:
            messagebox.showerror("Serial connect failed", "No COM port selected.")
            return

        try:
            self.worker.connect(port, self.baud_var.get())
        except Exception as exc:
            messagebox.showerror("Serial connect failed", str(exc))
            return
        self.model.connected = True
        self.model.source = port

    def _start_slam(self) -> None:
        self._send_command("SLAM")

    def _start_map(self) -> None:
        self._send_command("START MAP")

    def _start_map_auto96(self) -> None:
        self._send_command("START MAP")
        self.after(200, lambda: self._send_command("96"))

    def _set_lidar_quality(self) -> None:
        try:
            quality = int(self.lidar_quality_var.get())
        except (tk.TclError, ValueError):
            quality = 30
        quality = max(0, min(63, quality))
        self.lidar_quality_var.set(quality)
        self._send_command(f"LIDAR QUALITY {quality}")

    def _send_command(self, command: str) -> None:
        self.worker.write_line(command)
        self.model.raw_log.append(f"> {command}")
        if len(self.model.raw_log) > LOG_LIMIT:
            del self.model.raw_log[: len(self.model.raw_log) - LOG_LIMIT]

    def _disconnect(self) -> None:
        self.worker.disconnect()
        self.model.connected = False
        self.model.source = "offline"

    def _choose_replay(self) -> None:
        path = filedialog.askopenfilename(title="Replay serial log", filetypes=[("Text logs", "*.txt *.log"), ("All files", "*.*")])
        if path:
            self._start_replay(path)

    def _start_replay(self, path: str) -> None:
        self.model.connected = False
        self.model.source = f"replay: {path}"
        self.worker.replay(path)

    def _tick(self) -> None:
        drained = 0
        while drained < 200:
            try:
                line = self.events.get_nowait()
            except queue.Empty:
                break
            self.parser.parse_line(line)
            drained += 1

        self._update_status()
        self._draw()
        self.after(33, self._tick)

    def _update_status(self) -> None:
        self.status_vars["source"].set(("connected " if self.model.connected else "") + self.model.source)
        self.status_vars["map"].set(
            f"MAP {self.model.map_state} {self.model.width}x{self.model.height} "
            f"cell={self.model.cell_mm}mm rev={self.model.revision} rows={self.model.rows_received} "
            f"free={self.model.free_cells} occ={self.model.occupied_cells} "
            f"q>={self.model.lidar_quality_min} bias={self.model.lidar_distance_bias_mm}mm txdrop={self.model.tx_drops}"
        )
        if self.model.pose.valid:
            self.status_vars["pose"].set(
                f"x={self.model.pose.x_mm}mm y={self.model.pose.y_mm}mm "
                f"heading={self.model.pose.heading_cdeg / 100:.1f}deg"
            )
        self.status_vars["slam"].set(
            f"{self.model.slam.state} reason={self.model.slam.reason or '-'} "
            f"front={self.model.slam.front_mm}mm cell={self.model.slam.cell} target={self.model.slam.target}"
        )
        self.status_vars["path"].set(
            f"seq={self.model.path_seq} len={len(self.model.path)} "
            f"index={self.model.slam.path_i}/{self.model.slam.path_len}"
        )
        self.status_vars["last"].set(self.model.last_line)
        self._refresh_log()

    def _refresh_log(self) -> None:
        self.log_box.configure(state=tk.NORMAL)
        self.log_box.delete("1.0", tk.END)
        self.log_box.insert(tk.END, "\n".join(self.model.raw_log[-80:]))
        self.log_box.configure(state=tk.DISABLED)
        self.log_box.see(tk.END)

    def _draw(self) -> None:
        self.canvas.delete("all")
        scale = min(CANVAS_SIZE / max(1, self.model.width), CANVAS_SIZE / max(1, self.model.height))

        colors = {
            "?": "#5f6873",
            ".": "#f3f5f7",
            "#": "#111217",
        }
        for y, row in enumerate(self.model.grid):
            y0 = y * scale
            for x, cell in enumerate(row):
                x0 = x * scale
                self.canvas.create_rectangle(x0, y0, x0 + scale + 0.5, y0 + scale + 0.5, outline="", fill=colors.get(cell, "#5f6873"))

        self._draw_path(scale)
        self._draw_pose(scale)

    def _draw_path(self, scale: float) -> None:
        if not self.model.path:
            return

        points = [self._cell_center(x, y, scale) for x, y in self.model.path]
        if len(points) >= 2:
            flat = [coord for point in points for coord in point]
            self.canvas.create_line(*flat, fill="#118df0", width=3, smooth=False)

        for idx, (px, py) in enumerate(points):
            radius = 3 if idx != self.model.slam.path_i else 6
            color = "#118df0" if idx != self.model.slam.path_i else "#ffb000"
            self.canvas.create_oval(px - radius, py - radius, px + radius, py + radius, fill=color, outline="")

        if self.model.slam.target is not None:
            tx, ty = self._cell_center(*self.model.slam.target, scale)
            self.canvas.create_rectangle(tx - 7, ty - 7, tx + 7, ty + 7, outline="#20c997", width=3)

    def _draw_pose(self, scale: float) -> None:
        pose = self.model.pose
        if not pose.valid:
            return

        cell_x = (pose.x_mm + (self.model.width * self.model.cell_mm) / 2.0) / self.model.cell_mm
        cell_y = ((self.model.height * self.model.cell_mm) / 2.0 - pose.y_mm) / self.model.cell_mm
        x = cell_x * scale
        y = cell_y * scale
        heading_rad = math.radians(pose.heading_cdeg / 100.0)
        arrow_len = max(16.0, scale * 2.2)
        x2 = x + math.cos(heading_rad) * arrow_len
        y2 = y - math.sin(heading_rad) * arrow_len

        state_color = "#e03131" if self.model.slam.state == "NO_PATH" else "#ff6b35"
        self.canvas.create_oval(x - 7, y - 7, x + 7, y + 7, fill=state_color, outline="#ffffff", width=2)
        self.canvas.create_line(x, y, x2, y2, fill=state_color, width=4, arrow=tk.LAST)

    @staticmethod
    def _cell_center(x: int, y: int, scale: float) -> Tuple[float, float]:
        return (x + 0.5) * scale, (y + 0.5) * scale

    def _on_close(self) -> None:
        self.worker.disconnect()
        self.destroy()


def main(argv: Optional[Iterable[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Visualize STM32 SLAM/A* serial output.")
    parser.add_argument("--port", default=DEFAULT_PORT, help="Bluetooth serial port, for example COM6")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--replay", help="Replay a saved serial log instead of opening a live port")
    args = parser.parse_args(argv)

    app = SlamVisualizer(port=args.port, baud=args.baud, replay_path=args.replay)
    app.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
