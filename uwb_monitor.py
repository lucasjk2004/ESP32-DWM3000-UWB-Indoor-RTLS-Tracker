#!/usr/bin/env python3
"""
uwb_monitor.py
================

Live distance monitor for the Chordinates UWB rig.

Connects to Anchor 1 over USB serial, decodes the binary protocol from
uwb_protocol.py, and shows a 4 (tags) x 6 (anchors) grid of live
distances with bar visualisations. Stale cells (no update in
STALE_MS) fade out so you can see at a glance which links are dropping.

Run:
    python uwb_monitor.py                 # auto-pick first port
    python uwb_monitor.py --port COM7
    python uwb_monitor.py --port /dev/tty.usbserial-XXX
    python uwb_monitor.py --replay  capture.bin    # offline replay

Dependencies:
    pip install pyserial
    tkinter ships with the Python standard library on every desktop OS.
"""

from __future__ import annotations

import argparse
import sys
import time
import tkinter as tk
from collections import deque
from tkinter import ttk
from typing import Optional

from uwb_protocol import (
    FIRST_ANCHOR_ID, FIRST_TAG_ID, NUM_ANCHORS, NUM_TAGS,
    FrameDecoder, SerialReader, TagFrame, TextLine,
    list_ports, open_serial,
)


# Visual configuration --------------------------------------------------

BG_DARK    = "#0e1116"
BG_PANEL   = "#161a22"
BG_CELL    = "#1f2530"
BG_CELL_HOT = "#243043"
FG_PRIMARY = "#e6edf3"
FG_DIM     = "#7d8590"
FG_STALE   = "#3a4150"
ACCENT_BAR = "#3fb950"
ACCENT_BAD = "#f85149"
ACCENT_WARN = "#d29922"

FONT_MONO_LG = ("SF Mono", 18, "bold")
FONT_MONO_MD = ("SF Mono", 11)
FONT_MONO_SM = ("SF Mono", 9)
FONT_UI      = ("Helvetica", 11)
FONT_UI_BOLD = ("Helvetica", 11, "bold")

CELL_W = 130
CELL_H = 70
MAX_RANGE_M = 15.0    # bar saturates at this distance
STALE_MS = 500        # cells fade after this many ms with no update
REDRAW_HZ = 30        # GUI refresh rate


# ----------------------------------------------------------------------

class Cell:
    """One (tag, anchor) cell. Owns its own canvas items."""

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
        self.fresh: bool = False

    def update(self, distance_m: float, rssi_dbm: float, fresh: bool, now_ms: float):
        self.distance_m = distance_m
        self.rssi_dbm = rssi_dbm
        self.fresh = fresh
        self.last_update_ms = now_ms

    def render(self, now_ms: float):
        if self.last_update_ms is None or self.distance_m is None:
            self.canvas.itemconfig(self.dist_text, text="—", fill=FG_DIM)
            self.canvas.itemconfig(self.rssi_text, text="")
            self.canvas.coords(self.bar,
                self.x + 6, self.y + CELL_H - 8,
                self.x + 6, self.y + CELL_H - 4)
            self.canvas.itemconfig(self.bg, fill=BG_CELL)
            return

        age = now_ms - self.last_update_ms
        stale = age > STALE_MS
        very_stale = age > 2 * STALE_MS

        # text colour
        if very_stale:
            text_color = FG_STALE
        elif stale or not self.fresh:
            text_color = FG_DIM
        else:
            text_color = FG_PRIMARY

        # distance display: keep showing the last value, just fade it
        self.canvas.itemconfig(self.dist_text,
            text=f"{self.distance_m:5.2f}",
            fill=text_color,
        )
        self.canvas.itemconfig(self.unit_text,
            fill=text_color if not very_stale else FG_STALE)

        # rssi line
        rssi_str = f"{self.rssi_dbm:+.0f} dBm"
        self.canvas.itemconfig(self.rssi_text, text=rssi_str,
            fill=FG_STALE if very_stale else FG_DIM)

        # bar
        bar_w = max(0.0, min(1.0, self.distance_m / MAX_RANGE_M))
        bar_px = int(bar_w * (CELL_W - 12))
        if bar_px < 1:
            bar_px = 1
        bar_color = ACCENT_BAR
        if very_stale:
            bar_color = FG_STALE
        elif stale:
            bar_color = ACCENT_WARN
        elif self.distance_m > MAX_RANGE_M * 0.85:
            bar_color = ACCENT_WARN
        self.canvas.coords(self.bar,
            self.x + 6, self.y + CELL_H - 8,
            self.x + 6 + bar_px, self.y + CELL_H - 4)
        self.canvas.itemconfig(self.bar, fill=bar_color)

        # cell background pulse on fresh data
        if not very_stale and self.fresh and age < 80:
            self.canvas.itemconfig(self.bg, fill=BG_CELL_HOT)
        else:
            self.canvas.itemconfig(self.bg, fill=BG_CELL)


class TagRow:
    """Header label for a tag plus stats."""
    def __init__(self, canvas: tk.Canvas, tag_id: int, x: int, y: int):
        self.tag_id = tag_id
        canvas.create_text(x + 8, y + 14, anchor="w",
            text=f"TAG {tag_id}", fill=FG_PRIMARY, font=FONT_UI_BOLD)
        self.rate_text = canvas.create_text(x + 8, y + 38, anchor="w",
            text="0.0 Hz", fill=FG_DIM, font=FONT_MONO_SM)
        self.drop_text = canvas.create_text(x + 8, y + 54, anchor="w",
            text="seq —", fill=FG_DIM, font=FONT_MONO_SM)
        self.canvas = canvas

        self.recent_packets = deque(maxlen=64)   # timestamps
        self.last_seq: Optional[int] = None
        self.dropped = 0

    def saw_packet(self, seq: int, now_s: float):
        self.recent_packets.append(now_s)
        if self.last_seq is not None:
            expected = (self.last_seq + 1) & 0xFF
            if seq != expected:
                # Estimate drops; wraparound-safe for small gaps.
                gap = (seq - self.last_seq - 1) & 0xFF
                if gap < 64:  # ignore implausible huge gaps (likely re-sync)
                    self.dropped += gap
        self.last_seq = seq

    def render(self, now_s: float):
        # rate over last 2 seconds
        cutoff = now_s - 2.0
        while self.recent_packets and self.recent_packets[0] < cutoff:
            self.recent_packets.popleft()
        rate = len(self.recent_packets) / 2.0
        self.canvas.itemconfig(self.rate_text, text=f"{rate:4.1f} Hz")

        if self.last_seq is None:
            self.canvas.itemconfig(self.drop_text, text="seq —", fill=FG_DIM)
        else:
            color = FG_DIM if self.dropped == 0 else ACCENT_WARN
            self.canvas.itemconfig(self.drop_text,
                text=f"drops {self.dropped}",
                fill=color)


# ----------------------------------------------------------------------

class App:
    HEADER_H = 70
    TAG_LABEL_W = 110
    DEBUG_H = 130

    def __init__(self, root: tk.Tk, port: Optional[str] = None,
                 replay_file: Optional[str] = None):
        self.root = root
        self.root.title("UWB Monitor — Chordinates")
        self.root.configure(bg=BG_DARK)

        # Sizing
        canvas_w = self.TAG_LABEL_W + NUM_ANCHORS * CELL_W + 20
        canvas_h = self.HEADER_H + NUM_TAGS * CELL_H + 10
        total_h = canvas_h + self.DEBUG_H + 30

        self.root.geometry(f"{canvas_w + 20}x{total_h}")
        self.root.minsize(canvas_w + 20, total_h)

        # ---- top status bar ----
        status = tk.Frame(root, bg=BG_PANEL, height=28)
        status.pack(side="top", fill="x")
        self.status_left = tk.Label(status, text="• disconnected",
            font=FONT_UI, bg=BG_PANEL, fg=ACCENT_BAD, padx=12)
        self.status_left.pack(side="left")
        self.status_right = tk.Label(status, text="0 frames | 0 errors | 0 B/s",
            font=FONT_UI, bg=BG_PANEL, fg=FG_DIM, padx=12)
        self.status_right.pack(side="right")

        # ---- main canvas ----
        self.canvas = tk.Canvas(root, width=canvas_w, height=canvas_h,
            bg=BG_DARK, highlightthickness=0)
        self.canvas.pack(side="top", fill="both", expand=False, padx=10, pady=10)

        # column headers (anchors)
        for ai in range(NUM_ANCHORS):
            x = self.TAG_LABEL_W + ai * CELL_W
            self.canvas.create_text(x + CELL_W // 2, self.HEADER_H - 18,
                text=f"ANCHOR {FIRST_ANCHOR_ID + ai}",
                fill=FG_PRIMARY, font=FONT_UI_BOLD)
            self.canvas.create_line(x, self.HEADER_H - 4, x + CELL_W, self.HEADER_H - 4,
                fill=BG_PANEL)

        # title
        self.canvas.create_text(8, 14, anchor="w",
            text="LIVE DISTANCES",
            fill=FG_DIM, font=FONT_UI_BOLD)
        self.canvas.create_text(8, 34, anchor="w",
            text="metres • EMA filtered",
            fill=FG_DIM, font=FONT_MONO_SM)

        # rows + cells
        self.tag_rows: list[TagRow] = []
        # cells[tag_idx][anchor_idx]
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

        # ---- debug pane ----
        debug_frame = tk.Frame(root, bg=BG_PANEL)
        debug_frame.pack(side="bottom", fill="both", expand=True, padx=10, pady=(0, 10))

        debug_header = tk.Frame(debug_frame, bg=BG_PANEL)
        debug_header.pack(side="top", fill="x")
        tk.Label(debug_header, text="DEBUG", bg=BG_PANEL, fg=FG_DIM,
            font=FONT_UI_BOLD, padx=8, pady=4).pack(side="left")
        tk.Button(debug_header, text="Clear",
            command=self._clear_debug,
            bg=BG_CELL, fg=FG_PRIMARY, relief="flat",
            font=FONT_UI, padx=10).pack(side="right", padx=4, pady=2)

        self.debug_text = tk.Text(debug_frame, height=8,
            bg=BG_CELL, fg=FG_DIM, font=FONT_MONO_SM,
            relief="flat", borderwidth=0, wrap="none")
        self.debug_text.pack(side="bottom", fill="both", expand=True)
        self.debug_text.configure(state="disabled")

        # ---- runtime state ----
        self._running = True
        self._serial = None
        self._reader: Optional[SerialReader] = None
        self._replay_stats = None  # FrameStats from replay decoder, if any
        self._error: Optional[str] = None
        self._last_byte_count = 0
        self._last_byte_count_t = time.monotonic()
        self._byte_rate = 0.0
        # Pending events from background thread; consumed on the GUI thread.
        # We use a thread-safe deque rather than a Queue for slightly less
        # overhead on this hot path.
        self._pending: deque = deque()
        self._pending_lock = __import__("threading").Lock()

        # ---- start IO ----
        if replay_file:
            self._start_replay(replay_file)
        else:
            self._start_serial(port)

        # ---- redraw loop ----
        self._tick()

        root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ---- event handling ----

    def _on_serial_event(self, event):
        # Called from background thread.
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
            self._serial = open_serial(port, baud=115200, timeout=0.05)
        except Exception as e:
            self._set_status_disconnected(f"open failed: {e}")
            return

        self._reader = SerialReader(
            self._serial,
            on_event=self._on_serial_event,
            on_error=self._on_serial_error,
        )
        self._reader.start()
        self._set_status_connected(port)

    def _start_replay(self, path: str):
        """Replay a captured byte-stream file at real time-ish speed."""
        import threading
        try:
            with open(path, "rb") as f:
                data = f.read()
        except Exception as e:
            self._set_status_disconnected(f"replay open failed: {e}")
            return

        decoder = FrameDecoder()
        self._reader = None  # not a SerialReader; emulated
        self._replay_stats = decoder.stats

        def feeder():
            i = 0
            chunk = 64
            while self._running and i < len(data):
                events = list(decoder.feed(data[i:i + chunk]))
                with self._pending_lock:
                    for ev in events:
                        self._pending.append(ev)
                i += chunk
                time.sleep(0.005)
            self._log_debug(f"# replay finished: {len(data)} bytes")

        self._set_status_connected(f"replay:{path}")
        threading.Thread(target=feeder, daemon=True).start()

    def _set_status_connected(self, port: str):
        self.status_left.config(text=f"● connected · {port}", fg=ACCENT_BAR)

    def _set_status_disconnected(self, reason: str):
        self.status_left.config(text=f"○ {reason}", fg=ACCENT_BAD)

    # ---- redraw / poll ----

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
                if 0 <= ti < NUM_TAGS:
                    self.tag_rows[ti].saw_packet(ev.seq, now_s)
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
        now_s  = now_ms / 1000.0

        # render cells + headers
        for row in self.tag_rows:
            row.render(now_s)
        for row in self.cells:
            for cell in row:
                cell.render(now_ms)

        # status counts
        stats_obj = None
        if self._reader is not None:
            stats_obj = self._reader.stats
        elif self._replay_stats is not None:
            stats_obj = self._replay_stats

        if stats_obj is not None:
            now = time.monotonic()
            dt = now - self._last_byte_count_t
            if dt >= 0.5:
                self._byte_rate = (stats_obj.bytes_in - self._last_byte_count) / dt
                self._last_byte_count = stats_obj.bytes_in
                self._last_byte_count_t = now
            self.status_right.config(
                text=f"{stats_obj.good} frames · {stats_obj.bad_crc} CRC errs · {self._byte_rate:5.0f} B/s"
            )

        self.root.after(int(1000 / REDRAW_HZ), self._tick)

    # ---- debug pane ----

    def _log_debug(self, line: str):
        self.debug_text.configure(state="normal")
        self.debug_text.insert("end", line + "\n")
        # cap pane to last ~500 lines
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
    ap = argparse.ArgumentParser(description="UWB live distance monitor")
    ap.add_argument("--port", help="Serial port (auto-detected if omitted)")
    ap.add_argument("--replay", help="Replay a captured .bin file instead of opening serial")
    ap.add_argument("--list", action="store_true", help="List serial ports and exit")
    args = ap.parse_args()

    if args.list:
        for dev, desc in list_ports():
            print(f"{dev}\t{desc}")
        return 0

    root = tk.Tk()
    App(root, port=args.port, replay_file=args.replay)
    root.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
