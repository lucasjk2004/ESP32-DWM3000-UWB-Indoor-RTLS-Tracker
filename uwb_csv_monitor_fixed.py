#!/usr/bin/env python3
"""
uwb_csv_monitor.py
==================

Dark dashboard-style live distance monitor for your ESP32/DWM3000 listener anchor.

This version is designed for the CSV output from main_anchor1_listener.cpp:

    tag_id,dist0,dist1,dist2,dist3,dist4,dist5,rssi0,rssi1,rssi2,rssi3,rssi4,rssi5

Lines starting with # are shown in the debug panel and ignored as data.

Run:
    python uwb_csv_monitor.py
    python uwb_csv_monitor.py --port /dev/cu.usbserial-XXXX
    python uwb_csv_monitor.py --list

Dependencies:
    pip install pyserial

tkinter usually ships with desktop Python.
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
import threading
import tkinter as tk
from collections import deque
from dataclasses import dataclass
from tkinter import ttk
from typing import Optional

import serial
import serial.tools.list_ports


# ----------------------------------------------------------------------
# Rig configuration
# ----------------------------------------------------------------------

FIRST_ANCHOR_ID = 0
FIRST_TAG_ID = 7          # Change to 8 if your tags are 8,9,10,11
NUM_ANCHORS = 6
NUM_TAGS = 4
BAUD = 115200


# ----------------------------------------------------------------------
# Visual configuration
# ----------------------------------------------------------------------

BG_DARK      = "#0e1116"
BG_PANEL     = "#161a22"
BG_CELL      = "#1f2530"
BG_CELL_HOT  = "#243043"
FG_PRIMARY   = "#e6edf3"
FG_DIM       = "#7d8590"
FG_STALE     = "#3a4150"
ACCENT_BAR   = "#3fb950"
ACCENT_BAD   = "#f85149"
ACCENT_WARN  = "#d29922"

FONT_MONO_LG = ("SF Mono", 18, "bold")
FONT_MONO_MD = ("SF Mono", 11)
FONT_MONO_SM = ("SF Mono", 9)
FONT_UI      = ("Helvetica", 11)
FONT_UI_BOLD = ("Helvetica", 11, "bold")

CELL_W = 130
CELL_H = 70
MAX_RANGE_M = 15.0
STALE_MS = 500
REDRAW_HZ = 30


# ----------------------------------------------------------------------
# Data objects
# ----------------------------------------------------------------------

@dataclass
class TextLine:
    text: str


@dataclass
class TagFrame:
    tag_id: int
    distances_m: list[float]
    rssi_dbm: list[float]
    raw: str

    def is_fresh(self, anchor_idx: int) -> bool:
        if anchor_idx >= len(self.distances_m) or anchor_idx >= len(self.rssi_dbm):
            return False
        d = self.distances_m[anchor_idx]
        r = self.rssi_dbm[anchor_idx]
        return d > 0.001 and r != 0.0


@dataclass
class FrameStats:
    good: int = 0
    bad: int = 0
    bytes_in: int = 0


def list_ports():
    return [(p.device, p.description) for p in serial.tools.list_ports.comports()]


def open_serial(port: str, baud: int = BAUD, timeout: float = 0.05):
    return serial.Serial(port, baudrate=baud, timeout=timeout)


class CSVSerialReader(threading.Thread):
    """Reads CSV lines from the ESP32 serial port in a background thread."""

    def __init__(self, ser: serial.Serial, on_event, on_error):
        super().__init__(daemon=True)
        self.ser = ser
        self.on_event = on_event
        self.on_error = on_error
        self.stats = FrameStats()
        self._running = True

    def stop(self):
        self._running = False

    def run(self):
        try:
            time.sleep(1.5)  # ESP32 often resets when serial opens
            while self._running:
                raw = self.ser.readline()
                if not raw:
                    continue

                self.stats.bytes_in += len(raw)
                line = raw.decode("utf-8", errors="replace").strip()

                if not line:
                    continue

                if line.startswith("#"):
                    self.on_event(TextLine(line))
                    continue

                frame = self.parse_line(line)
                if frame is None:
                    self.stats.bad += 1
                    self.on_event(TextLine(f"[bad csv] {line}"))
                else:
                    self.stats.good += 1
                    self.on_event(frame)

        except Exception as e:
            self.on_error(e)

    @staticmethod
    def parse_line(line: str) -> Optional[TagFrame]:
        try:
            row = next(csv.reader([line]))

            needed_cols = 1 + NUM_ANCHORS + NUM_ANCHORS
            if len(row) < needed_cols:
                return None

            tag_id = int(float(row[0]))
            distances = [float(x) for x in row[1:1 + NUM_ANCHORS]]
            rssi = [float(x) for x in row[1 + NUM_ANCHORS:1 + NUM_ANCHORS + NUM_ANCHORS]]

            return TagFrame(
                tag_id=tag_id,
                distances_m=distances,
                rssi_dbm=rssi,
                raw=line,
            )
        except Exception:
            return None


# ----------------------------------------------------------------------
# Visual cells
# ----------------------------------------------------------------------

class Cell:
    """One tag-anchor cell."""

    def __init__(self, canvas: tk.Canvas, x: int, y: int):
        self.canvas = canvas
        self.x = x
        self.y = y

        self.bg = canvas.create_rectangle(
            x, y, x + CELL_W, y + CELL_H,
            fill=BG_CELL, outline=BG_PANEL, width=1,
        )
        self.bar = canvas.create_rectangle(
            x + 6, y + CELL_H - 8, x + 6, y + CELL_H - 4,
            fill=ACCENT_BAR, outline="",
        )
        self.dist_text = canvas.create_text(
            x + 12, y + 12, anchor="nw",
            text="—", fill=FG_DIM, font=FONT_MONO_LG,
        )
        self.unit_text = canvas.create_text(
            x + CELL_W - 8, y + 18, anchor="ne",
            text="m", fill=FG_DIM, font=FONT_MONO_SM,
        )
        self.rssi_text = canvas.create_text(
            x + 12, y + CELL_H - 22, anchor="nw",
            text="", fill=FG_DIM, font=FONT_MONO_SM,
        )

        self.last_update_ms: Optional[float] = None
        self.distance_m: Optional[float] = None
        self.rssi_dbm: Optional[float] = None
        self.fresh = False

    def update(self, distance_m: float, rssi_dbm: float, fresh: bool, now_ms: float):
        self.distance_m = distance_m
        self.rssi_dbm = rssi_dbm
        self.fresh = fresh
        self.last_update_ms = now_ms

    def render(self, now_ms: float):
        if self.last_update_ms is None or self.distance_m is None:
            self.canvas.itemconfig(self.dist_text, text="—", fill=FG_DIM)
            self.canvas.itemconfig(self.rssi_text, text="")
            self.canvas.coords(
                self.bar,
                self.x + 6, self.y + CELL_H - 8,
                self.x + 6, self.y + CELL_H - 4,
            )
            self.canvas.itemconfig(self.bg, fill=BG_CELL)
            return

        age = now_ms - self.last_update_ms
        stale = age > STALE_MS
        very_stale = age > 2 * STALE_MS

        if very_stale:
            text_color = FG_STALE
        elif stale or not self.fresh:
            text_color = FG_DIM
        else:
            text_color = FG_PRIMARY

        if self.distance_m <= 0.001:
            dist_display = "  —  "
        else:
            dist_display = f"{self.distance_m:5.2f}"

        self.canvas.itemconfig(self.dist_text, text=dist_display, fill=text_color)
        self.canvas.itemconfig(self.unit_text, fill=text_color if not very_stale else FG_STALE)

        if self.rssi_dbm is None or self.rssi_dbm == 0:
            rssi_str = "no RSSI"
        else:
            rssi_str = f"{self.rssi_dbm:+.0f} dBm"

        self.canvas.itemconfig(
            self.rssi_text,
            text=rssi_str,
            fill=FG_STALE if very_stale else FG_DIM,
        )

        bar_w = max(0.0, min(1.0, self.distance_m / MAX_RANGE_M if self.distance_m else 0.0))
        bar_px = max(1, int(bar_w * (CELL_W - 12)))

        bar_color = ACCENT_BAR
        if very_stale:
            bar_color = FG_STALE
        elif stale:
            bar_color = ACCENT_WARN
        elif self.distance_m and self.distance_m > MAX_RANGE_M * 0.85:
            bar_color = ACCENT_WARN
        elif not self.fresh:
            bar_color = FG_DIM

        self.canvas.coords(
            self.bar,
            self.x + 6, self.y + CELL_H - 8,
            self.x + 6 + bar_px, self.y + CELL_H - 4,
        )
        self.canvas.itemconfig(self.bar, fill=bar_color)

        if not very_stale and self.fresh and age < 80:
            self.canvas.itemconfig(self.bg, fill=BG_CELL_HOT)
        else:
            self.canvas.itemconfig(self.bg, fill=BG_CELL)


class TagRow:
    """Left-side tag label plus packet rate estimate."""

    def __init__(self, canvas: tk.Canvas, tag_id: int, x: int, y: int):
        self.tag_id = tag_id
        self.canvas = canvas

        canvas.create_text(
            x + 8, y + 14, anchor="w",
            text=f"TAG {tag_id}", fill=FG_PRIMARY, font=FONT_UI_BOLD,
        )
        self.rate_text = canvas.create_text(
            x + 8, y + 38, anchor="w",
            text="0.0 Hz", fill=FG_DIM, font=FONT_MONO_SM,
        )
        self.last_text = canvas.create_text(
            x + 8, y + 54, anchor="w",
            text="no data", fill=FG_DIM, font=FONT_MONO_SM,
        )

        self.recent_packets = deque(maxlen=128)
        self.last_seen_s: Optional[float] = None

    def saw_packet(self, now_s: float):
        self.recent_packets.append(now_s)
        self.last_seen_s = now_s

    def render(self, now_s: float):
        cutoff = now_s - 2.0
        while self.recent_packets and self.recent_packets[0] < cutoff:
            self.recent_packets.popleft()

        rate = len(self.recent_packets) / 2.0
        self.canvas.itemconfig(self.rate_text, text=f"{rate:4.1f} Hz")

        if self.last_seen_s is None:
            self.canvas.itemconfig(self.last_text, text="no data", fill=FG_DIM)
        else:
            age = now_s - self.last_seen_s
            if age < 1:
                self.canvas.itemconfig(self.last_text, text="live", fill=ACCENT_BAR)
            else:
                self.canvas.itemconfig(self.last_text, text=f"{age:3.1f}s ago", fill=ACCENT_WARN)


# ----------------------------------------------------------------------
# Main app
# ----------------------------------------------------------------------

class App:
    HEADER_H = 70
    TAG_LABEL_W = 110
    DEBUG_H = 130

    def __init__(self, root: tk.Tk, port: Optional[str] = None):
        self.root = root
        self.root.title("UWB Monitor — CSV Listener")
        self.root.configure(bg=BG_DARK)

        canvas_w = self.TAG_LABEL_W + NUM_ANCHORS * CELL_W + 20
        canvas_h = self.HEADER_H + NUM_TAGS * CELL_H + 10
        total_h = canvas_h + self.DEBUG_H + 30

        self.root.geometry(f"{canvas_w + 20}x{total_h}")
        self.root.minsize(canvas_w + 20, total_h)

        # Top status bar
        status = tk.Frame(root, bg=BG_PANEL, height=28)
        status.pack(side="top", fill="x")

        self.status_left = tk.Label(
            status, text="• disconnected",
            font=FONT_UI, bg=BG_PANEL, fg=ACCENT_BAD, padx=12,
        )
        self.status_left.pack(side="left")

        self.status_right = tk.Label(
            status, text="0 frames · 0 bad · 0 B/s",
            font=FONT_UI, bg=BG_PANEL, fg=FG_DIM, padx=12,
        )
        self.status_right.pack(side="right")

        # Main canvas
        self.canvas = tk.Canvas(
            root, width=canvas_w, height=canvas_h,
            bg=BG_DARK, highlightthickness=0,
        )
        self.canvas.pack(side="top", fill="both", expand=False, padx=10, pady=10)

        # Anchor column headers
        for ai in range(NUM_ANCHORS):
            x = self.TAG_LABEL_W + ai * CELL_W
            self.canvas.create_text(
                x + CELL_W // 2, self.HEADER_H - 18,
                text=f"ANCHOR {FIRST_ANCHOR_ID + ai}",
                fill=FG_PRIMARY, font=FONT_UI_BOLD,
            )
            self.canvas.create_line(
                x, self.HEADER_H - 4, x + CELL_W, self.HEADER_H - 4,
                fill=BG_PANEL,
            )

        self.canvas.create_text(
            8, 14, anchor="w",
            text="LIVE DISTANCES",
            fill=FG_DIM, font=FONT_UI_BOLD,
        )
        self.canvas.create_text(
            8, 34, anchor="w",
            text="metres • CSV serial",
            fill=FG_DIM, font=FONT_MONO_SM,
        )

        self.tag_rows: list[TagRow] = []
        self.cells: list[list[Cell]] = []

        for ti in range(NUM_TAGS):
            tag_id = FIRST_TAG_ID + ti
            y = self.HEADER_H + ti * CELL_H
            self.tag_rows.append(TagRow(self.canvas, tag_id, 0, y))

            row_cells = []
            for ai in range(NUM_ANCHORS):
                x = self.TAG_LABEL_W + ai * CELL_W
                row_cells.append(Cell(self.canvas, x + 4, y + 4))
            self.cells.append(row_cells)

        # Debug pane
        debug_frame = tk.Frame(root, bg=BG_PANEL)
        debug_frame.pack(side="bottom", fill="both", expand=True, padx=10, pady=(0, 10))

        debug_header = tk.Frame(debug_frame, bg=BG_PANEL)
        debug_header.pack(side="top", fill="x")

        tk.Label(
            debug_header, text="DEBUG",
            bg=BG_PANEL, fg=FG_DIM, font=FONT_UI_BOLD, padx=8, pady=4,
        ).pack(side="left")

        tk.Button(
            debug_header, text="Clear",
            command=self._clear_debug,
            bg=BG_CELL, fg=FG_PRIMARY, relief="flat",
            font=FONT_UI, padx=10,
        ).pack(side="right", padx=4, pady=2)

        self.debug_text = tk.Text(
            debug_frame, height=8,
            bg=BG_CELL, fg=FG_DIM, font=FONT_MONO_SM,
            relief="flat", borderwidth=0, wrap="none",
        )
        self.debug_text.pack(side="bottom", fill="both", expand=True)
        self.debug_text.configure(state="disabled")

        # Runtime state
        self._running = True
        self._serial = None
        self._reader: Optional[CSVSerialReader] = None
        self._last_byte_count = 0
        self._last_byte_count_t = time.monotonic()
        self._byte_rate = 0.0
        self._pending: deque = deque()
        self._pending_lock = threading.Lock()

        self._start_serial(port)
        self._tick()

        root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _on_serial_event(self, event):
        with self._pending_lock:
            self._pending.append(event)

    def _on_serial_error(self, err: Exception):
        with self._pending_lock:
            self._pending.append(("__error__", err))

    def _start_serial(self, port: Optional[str]):
        if port is None:
            ports = list_ports()
            if not ports:
                self._set_status_disconnected("no serial ports found")
                return
            port = ports[0][0]
            self._log_debug(f"# auto-selected port: {port}")

        try:
            self._serial = open_serial(port, baud=BAUD, timeout=0.05)
        except Exception as e:
            self._set_status_disconnected(f"open failed: {e}")
            return

        self._reader = CSVSerialReader(
            self._serial,
            on_event=self._on_serial_event,
            on_error=self._on_serial_error,
        )
        self._reader.start()
        self._set_status_connected(port)

    def _set_status_connected(self, port: str):
        self.status_left.config(text=f"● connected · {port}", fg=ACCENT_BAR)

    def _set_status_disconnected(self, reason: str):
        self.status_left.config(text=f"○ {reason}", fg=ACCENT_BAD)

    def _drain_pending(self):
        with self._pending_lock:
            events = list(self._pending)
            self._pending.clear()

        now_ms = time.monotonic() * 1000.0
        now_s = now_ms / 1000.0

        for ev in events:
            if isinstance(ev, tuple) and ev and ev[0] == "__error__":
                self._set_status_disconnected(f"io error: {ev[1]}")
                continue

            if isinstance(ev, TextLine):
                self._log_debug(ev.text)
                continue

            if isinstance(ev, TagFrame):
                ti = ev.tag_id - FIRST_TAG_ID

                if not (0 <= ti < NUM_TAGS):
                    self._log_debug(f"[tag outside grid] {ev.raw}")
                    continue

                self.tag_rows[ti].saw_packet(now_s)

                for ai in range(NUM_ANCHORS):
                    self.cells[ti][ai].update(
                        ev.distances_m[ai],
                        ev.rssi_dbm[ai],
                        ev.is_fresh(ai),
                        now_ms,
                    )

    def _tick(self):
        if not self._running:
            return

        self._drain_pending()

        now_ms = time.monotonic() * 1000.0
        now_s = now_ms / 1000.0

        for row in self.tag_rows:
            row.render(now_s)

        for row in self.cells:
            for cell in row:
                cell.render(now_ms)

        if self._reader is not None:
            stats = self._reader.stats
            now = time.monotonic()
            dt = now - self._last_byte_count_t

            if dt >= 0.5:
                self._byte_rate = (stats.bytes_in - self._last_byte_count) / dt
                self._last_byte_count = stats.bytes_in
                self._last_byte_count_t = now

            self.status_right.config(
                text=f"{stats.good} frames · {stats.bad} bad · {self._byte_rate:5.0f} B/s"
            )

        self.root.after(int(1000 / REDRAW_HZ), self._tick)

    def _log_debug(self, line: str):
        self.debug_text.configure(state="normal")
        self.debug_text.insert("end", line + "\n")

        line_count = int(self.debug_text.index("end-1c").split(".")[0])
        if line_count > 500:
            self.debug_text.delete("1.0", f"{line_count - 500}.0")

        self.debug_text.see("end")
        self.debug_text.configure(state="disabled")

    def _clear_debug(self):
        self.debug_text.configure(state="normal")
        self.debug_text.delete("1.0", "end")
        self.debug_text.configure(state="disabled")

    def _on_close(self):
        self._running = False
        try:
            if self._reader:
                self._reader.stop()
        except Exception:
            pass
        try:
            if self._serial:
                self._serial.close()
        except Exception:
            pass
        self.root.destroy()


# ----------------------------------------------------------------------

def main():
    global FIRST_TAG_ID, FIRST_ANCHOR_ID

    ap = argparse.ArgumentParser(description="UWB CSV live distance monitor")
    ap.add_argument("--port", help="Serial port. Auto-detected if omitted.")
    ap.add_argument("--list", action="store_true", help="List serial ports and exit.")
    ap.add_argument("--first-tag", type=int, default=FIRST_TAG_ID, help="First tag ID shown in the grid.")
    ap.add_argument("--first-anchor", type=int, default=FIRST_ANCHOR_ID, help="First anchor ID shown in the grid.")
    args = ap.parse_args()

    FIRST_TAG_ID = args.first_tag
    FIRST_ANCHOR_ID = args.first_anchor

    if args.list:
        for dev, desc in list_ports():
            print(f"{dev}\t{desc}")
        return 0

    root = tk.Tk()
    App(root, port=args.port)
    root.mainloop()
    return 0

if __name__ == "__main__":
    sys.exit(main())
