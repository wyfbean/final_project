#!/usr/bin/env python3
"""Minimal Go/Back control panel for the STM32 SLAM firmware.

The desktop app only sends text commands and displays firmware telemetry.
SLAM, A*, obstacle handling, and return-home decisions run entirely on the MCU.
"""

from __future__ import annotations

import argparse
import queue
import re
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk
from typing import Optional

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover
    serial = None
    list_ports = None


DEFAULT_PORT = "COM6"
DEFAULT_BAUD = 921600
LOG_LIMIT = 500
BLUETOOTH_KEYWORDS = ("bluetooth", "standard serial over bluetooth", "bth", "spp")

SLAM_HB_RE = re.compile(
    r"SLAM HB state=(?P<state>\S+)\s+seq=(?P<seq>\d+)\s+target=(?P<target>\d+,\d+)\s+"
    r"path_i=(?P<path_i>\d+)\s+path_len=(?P<path_len>\d+)\s+front=(?P<front>\d+)\s+rev=(?P<rev>\d+)"
)
SLAM_STATE_RE = re.compile(r"SLAM state=(?P<state>\S+)\s+reason=(?P<reason>\S+)\s+seq=(?P<seq>\d+).*")
MAP_STAT_RE = re.compile(r"MAP STAT\s+(?P<body>.*)")


def list_serial_port_labels() -> list[str]:
    if list_ports is None:
        return []

    labels: list[str] = []
    for port in list_ports.comports():
        details = " ".join(
            item for item in (port.description, port.manufacturer, port.hwid) if item
        )
        labels.append(f"{port.device}  {details}".strip())
    return labels


def extract_port_name(label: str) -> str:
    return label.strip().split()[0] if label.strip() else ""


def pick_bluetooth_port_label(labels: list[str]) -> Optional[str]:
    for label in labels:
        lower = label.lower()
        if any(keyword in lower for keyword in BLUETOOTH_KEYWORDS):
            return label
    return labels[0] if len(labels) == 1 else None


class SerialLink:
    def __init__(self, rx_queue: queue.Queue[str]) -> None:
        self.rx_queue = rx_queue
        self.serial_obj = None
        self.stop_event = threading.Event()
        self.thread: Optional[threading.Thread] = None

    @property
    def connected(self) -> bool:
        return self.serial_obj is not None and self.serial_obj.is_open

    def connect(self, port: str, baud: int) -> None:
        if serial is None:
            raise RuntimeError("pyserial is not installed. Run: python -m pip install pyserial")

        self.disconnect()
        self.stop_event.clear()
        self.serial_obj = serial.Serial(
            port=port,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.05,
            write_timeout=1.0,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )
        try:
            self.serial_obj.setDTR(False)
            self.serial_obj.setRTS(False)
        except Exception:
            pass
        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()

    def disconnect(self) -> None:
        self.stop_event.set()
        if self.serial_obj is not None:
            try:
                self.serial_obj.close()
            except Exception:
                pass
        self.serial_obj = None

    def send(self, command: str) -> None:
        if not self.connected:
            raise RuntimeError("serial port is not connected")

        payload = (command.strip() + "\r\n").encode("ascii", errors="ignore")
        self.serial_obj.write(payload)
        self.serial_obj.flush()
        self.rx_queue.put(f"[TX] {command.strip()}")

    def _read_loop(self) -> None:
        buffer = b""

        while not self.stop_event.is_set() and self.serial_obj is not None:
            try:
                data = self.serial_obj.read(1024)
            except Exception as exc:
                self.rx_queue.put(f"[ERROR] {exc}")
                break

            if not data:
                time.sleep(0.01)
                continue

            buffer += data
            while b"\n" in buffer:
                raw_line, buffer = buffer.split(b"\n", 1)
                line = raw_line.decode("ascii", errors="replace").strip()
                if line:
                    self.rx_queue.put(line)


class GoBackGui(tk.Tk):
    def __init__(self, port: str, baud: int) -> None:
        super().__init__()
        self.title("STM32 SLAM Go / Back")
        self.geometry("760x520")

        self.rx_queue: queue.Queue[str] = queue.Queue()
        self.link = SerialLink(self.rx_queue)
        self.port_var = tk.StringVar(value=port)
        self.baud_var = tk.IntVar(value=baud)
        self.connection_var = tk.StringVar(value="Disconnected")
        self.command_var = tk.StringVar(value="")
        self.slam_var = tk.StringVar(value="SLAM --")
        self.path_var = tk.StringVar(value="Path --")
        self.map_var = tk.StringVar(value="Map --")
        self.last_var = tk.StringVar(value="")
        self.log_lines: list[str] = []
        self.port_labels: list[str] = []

        self._build_ui()
        self._refresh_ports(auto_select=True)
        self.after(40, self._poll)
        self.protocol("WM_DELETE_WINDOW", self._close)

    def _build_ui(self) -> None:
        top = ttk.Frame(self)
        top.pack(side=tk.TOP, fill=tk.X, padx=10, pady=8)

        ttk.Label(top, text="Port").pack(side=tk.LEFT)
        self.port_combo = ttk.Combobox(top, textvariable=self.port_var, width=36)
        self.port_combo.pack(side=tk.LEFT, padx=(4, 8))
        ttk.Button(top, text="Refresh", command=lambda: self._refresh_ports(auto_select=False)).pack(side=tk.LEFT, padx=2)
        ttk.Button(top, text="Auto BT", command=self._auto_select_bluetooth).pack(side=tk.LEFT, padx=2)
        ttk.Label(top, text="Baud").pack(side=tk.LEFT)
        ttk.Entry(top, textvariable=self.baud_var, width=8).pack(side=tk.LEFT, padx=(4, 8))
        ttk.Button(top, text="Connect", command=self._connect).pack(side=tk.LEFT, padx=2)
        ttk.Button(top, text="Disconnect", command=self._disconnect).pack(side=tk.LEFT, padx=2)

        controls = ttk.Frame(self)
        controls.pack(side=tk.TOP, fill=tk.X, padx=10, pady=(0, 8))
        ttk.Button(controls, text="Go", command=lambda: self._send("SLAM")).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=3)
        ttk.Button(controls, text="Back", command=lambda: self._send("BACK")).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=3)
        ttk.Button(controls, text="0 Brake", command=lambda: self._send("0")).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=3)
        ttk.Button(controls, text="Stop", command=lambda: self._send("SLAM OFF")).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=3)
        ttk.Button(controls, text="96 Wall", command=lambda: self._send("96")).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=3)
        ttk.Button(controls, text="Gyro Cal", command=lambda: self._send("GYRO CAL")).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=3)
        ttk.Button(controls, text="Show Map", command=lambda: self._send("SHOW MAP")).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=3)

        command_row = ttk.Frame(self)
        command_row.pack(side=tk.TOP, fill=tk.X, padx=10, pady=(0, 8))
        ttk.Label(command_row, text="Command").pack(side=tk.LEFT)
        command_entry = ttk.Entry(command_row, textvariable=self.command_var)
        command_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(6, 6))
        command_entry.bind("<Return>", lambda _event: self._send_custom())
        ttk.Button(command_row, text="Send", command=self._send_custom).pack(side=tk.LEFT)

        status = ttk.Frame(self)
        status.pack(side=tk.TOP, fill=tk.X, padx=10, pady=(0, 8))
        for label, var in (
            ("Connection", self.connection_var),
            ("SLAM", self.slam_var),
            ("Path", self.path_var),
            ("Map", self.map_var),
            ("Last", self.last_var),
        ):
            row = ttk.Frame(status)
            row.pack(fill=tk.X, pady=2)
            ttk.Label(row, text=label, width=12).pack(side=tk.LEFT)
            ttk.Label(row, textvariable=var).pack(side=tk.LEFT, fill=tk.X, expand=True)

        self.log = tk.Text(self, height=18, state=tk.DISABLED)
        self.log.pack(fill=tk.BOTH, expand=True, padx=10, pady=(0, 10))

    def _refresh_ports(self, auto_select: bool) -> None:
        self.port_labels = list_serial_port_labels()
        self.port_combo.configure(values=self.port_labels)
        if auto_select:
            selected = pick_bluetooth_port_label(self.port_labels)
            if selected is not None:
                self.port_var.set(selected)
        self._append_log(f"[PORTS] {len(self.port_labels)} serial port(s)")

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
            messagebox.showerror("Connect failed", "No COM port selected.")
            return

        try:
            self.link.connect(port, self.baud_var.get())
        except Exception as exc:
            self._append_log(f"[ERROR] connect {port}: {exc}")
            messagebox.showerror("Connect failed", str(exc))
            return
        self.connection_var.set(f"Connected {port} @ {self.baud_var.get()}")
        self._append_log(f"[CONNECTED] {port} @ {self.baud_var.get()}")

    def _disconnect(self) -> None:
        self.link.disconnect()
        self.connection_var.set("Disconnected")
        self._append_log("[DISCONNECTED]")

    def _send(self, command: str) -> None:
        try:
            self.link.send(command)
        except Exception as exc:
            self._append_log(f"[ERROR] send {command}: {exc}")
            messagebox.showerror("Send failed", str(exc))

    def _send_custom(self) -> None:
        command = self.command_var.get().strip()
        if not command:
            return
        self._send(command)
        self.command_var.set("")

    def _poll(self) -> None:
        while True:
            try:
                line = self.rx_queue.get_nowait()
            except queue.Empty:
                break
            self._handle_line(line)
        self.after(40, self._poll)

    def _handle_line(self, line: str) -> None:
        self.last_var.set(line)
        self._append_log(line)

        if match := SLAM_HB_RE.match(line):
            self.slam_var.set(f"{match.group('state')} front={match.group('front')}mm rev={match.group('rev')}")
            self.path_var.set(f"seq={match.group('seq')} {match.group('path_i')}/{match.group('path_len')} target={match.group('target')}")
        elif match := SLAM_STATE_RE.match(line):
            self.slam_var.set(f"{match.group('state')} reason={match.group('reason')} seq={match.group('seq')}")
        elif match := MAP_STAT_RE.match(line):
            self.map_var.set(match.group("body"))

    def _append_log(self, line: str) -> None:
        self.log_lines.append(line)
        if len(self.log_lines) > LOG_LIMIT:
            del self.log_lines[: len(self.log_lines) - LOG_LIMIT]

        self.log.configure(state=tk.NORMAL)
        self.log.delete("1.0", tk.END)
        self.log.insert(tk.END, "\n".join(self.log_lines[-160:]))
        self.log.configure(state=tk.DISABLED)
        self.log.see(tk.END)

    def _close(self) -> None:
        self.link.disconnect()
        self.destroy()


def main() -> int:
    parser = argparse.ArgumentParser(description="Send Go/Back commands to STM32 SLAM firmware.")
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    args = parser.parse_args()

    app = GoBackGui(args.port, args.baud)
    app.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
