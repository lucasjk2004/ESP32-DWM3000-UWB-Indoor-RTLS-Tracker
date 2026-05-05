#!/usr/bin/env python3
"""
uwb_synth_bridge.py
====================

Reads UWB data from Anchor 1 (USB serial), trilaterates each tag's 3D
position using least squares + a per-tag Kalman filter, and forwards
smooth (x, y, z) coordinates to the synth over OSC.

Design notes
------------

* Trilateration: We do simple linearised least-squares using the first
  valid anchor as reference, like the guide. With 6 anchors we have
  5 equations for 3 unknowns (good overdetermination). We solve via
  numpy's pseudo-inverse for stability.

* Kalman: 6-state constant-velocity (x, y, z, vx, vy, vz). One
  filter per tag, instantiated lazily on first valid measurement.

* Smoothing trade-off (Q vs R): exposed on the command line so you
  can tune for the synth feel without re-editing code.

* Output: OSC messages
    /uwb/tag/<id>/pos   x y z      (filtered, metres)
    /uwb/tag/<id>/raw   x y z      (raw least-squares, metres)
    /uwb/tag/<id>/anchors  d1 d2 d3 d4 d5 d6   (metres)
    /uwb/tag/<id>/lost  1          (sent if we miss N consecutive cycles)

  These addresses are easy to map to synth params on the Electron side.
  If you don't have python-osc installed we fall back to printing JSON
  lines on stdout so you can pipe them into anything.

Anchor positions
----------------
Edit ANCHOR_POSITIONS below to match where you actually mount the 6
anchors in your space. Coordinates are metres in a right-handed frame
of your choice; the synth just needs consistency, not absolute truth.

Run:
    python uwb_synth_bridge.py --port /dev/tty.usbserial-XXX
    python uwb_synth_bridge.py --replay synthetic.bin --osc-host 127.0.0.1 --osc-port 9000
    python uwb_synth_bridge.py --no-osc        # print JSON instead
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from uwb_protocol import (
    FIRST_ANCHOR_ID, FIRST_TAG_ID, NUM_ANCHORS, NUM_TAGS,
    FrameDecoder, SerialReader, TagFrame, TextLine,
    open_serial, list_ports,
)


# ---- ANCHOR LAYOUT ----
# Edit to match your physical setup. (x, y, z) in metres.
# A reasonable default for a ~4x4x2.5m room with 6 anchors —
# 4 on the floor corners, 2 on the ceiling rail.
ANCHOR_POSITIONS: dict[int, tuple[float, float, float]] = {
    1: (0.0, 0.0, 0.20),
    2: (4.0, 0.0, 0.20),
    3: (4.0, 4.0, 0.20),
    4: (0.0, 4.0, 0.20),
    5: (1.0, 2.0, 2.40),
    6: (3.0, 2.0, 2.40),
}

# Per-anchor calibration offset (metres). Add to measured distance.
ANCHOR_OFFSETS: dict[int, float] = {1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0, 5: 0.0, 6: 0.0}


# ----------------------------------------------------------------------

class Kalman3D:
    """6-state constant-velocity Kalman filter. State: [x y z vx vy vz]."""

    def __init__(self, dt: float = 0.10, q: float = 0.05, r: float = 0.20):
        self.dt = dt
        # Process noise covariance
        self.Q = np.eye(6) * q
        # Measurement noise covariance (3x3, position only)
        self.R = np.eye(3) * r
        self.x = np.zeros(6)
        self.P = np.eye(6) * 1.0
        # Constant-velocity transition matrix
        self.F = np.eye(6)
        self.F[0, 3] = dt
        self.F[1, 4] = dt
        self.F[2, 5] = dt
        # Position observation matrix (3x6)
        self.H = np.zeros((3, 6))
        self.H[0, 0] = 1
        self.H[1, 1] = 1
        self.H[2, 2] = 1
        self.initialized = False

    def predict(self, dt: Optional[float] = None):
        if dt is not None and abs(dt - self.dt) > 1e-3:
            self.dt = dt
            self.F[0, 3] = dt; self.F[1, 4] = dt; self.F[2, 5] = dt
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, z: np.ndarray):
        if not self.initialized:
            self.x[:3] = z
            self.x[3:] = 0
            self.initialized = True
            return
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(6) - K @ self.H) @ self.P

    @property
    def position(self) -> np.ndarray:
        return self.x[:3].copy()


# ----------------------------------------------------------------------

def trilaterate(anchor_xyz: np.ndarray, distances: np.ndarray) -> Optional[np.ndarray]:
    """
    Linearised least-squares trilateration.

    anchor_xyz : (N, 3) float array of anchor positions
    distances  : (N,)   float array of measured distances (same units)

    Returns: (3,) tag position, or None if degenerate.

    Subtracts equation 0 from each other equation to eliminate the
    quadratic |p|^2 term, leaving a linear system in (x, y, z).
    """
    n = anchor_xyz.shape[0]
    if n < 4:
        return None  # need 4+ anchors for full 3D solution

    A = np.zeros((n - 1, 3))
    b = np.zeros(n - 1)
    p0 = anchor_xyz[0]
    r0 = distances[0]
    for i in range(1, n):
        pi = anchor_xyz[i]
        ri = distances[i]
        A[i - 1] = 2.0 * (pi - p0)
        b[i - 1] = (r0 ** 2 - ri ** 2) - (np.dot(p0, p0) - np.dot(pi, pi))

    try:
        # lstsq is more robust than naïve normal equations
        sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    except np.linalg.LinAlgError:
        return None
    return sol


# ----------------------------------------------------------------------

@dataclass
class TagState:
    tag_id: int
    kalman: Kalman3D = field(default_factory=Kalman3D)
    last_update_t: float = 0.0
    last_seq: int = -1
    consecutive_misses: int = 0
    raw_pos: Optional[np.ndarray] = None
    is_lost: bool = False


class OSCSink:
    """
    Sends OSC messages, OR prints JSON lines on stdout when OSC is disabled
    or python-osc isn't installed. Either way you get output you can pipe.
    """

    def __init__(self, host: str = "127.0.0.1", port: int = 9000, enabled: bool = True):
        self.enabled_osc = enabled
        self._client = None
        if enabled:
            try:
                from pythonosc.udp_client import SimpleUDPClient  # type: ignore
                self._client = SimpleUDPClient(host, port)
                print(f"[osc] -> {host}:{port}", file=sys.stderr)
            except ImportError:
                print("[osc] python-osc not installed; using JSON stdout instead.", file=sys.stderr)
                print("[osc]   pip install python-osc   # to enable OSC", file=sys.stderr)
        else:
            print("[osc] disabled; using JSON stdout instead.", file=sys.stderr)

    def send(self, address: str, *args):
        if self._client is not None:
            self._client.send_message(address, list(args))
        else:
            # Compact JSON line. Easy for any consumer to parse.
            print(json.dumps({"addr": address, "args": list(args)}), flush=True)


# ----------------------------------------------------------------------

class Bridge:
    LOST_THRESHOLD = 5  # cycles with no fresh data before we declare a tag lost

    def __init__(self, sink: OSCSink, kalman_q: float, kalman_r: float,
                 min_anchors: int = 4):
        self.sink = sink
        self.kalman_q = kalman_q
        self.kalman_r = kalman_r
        self.min_anchors = min_anchors
        self.tags: dict[int, TagState] = {}

        # Pre-compute anchor position matrix indexed by anchor INDEX (0..5).
        # Anchor IDs are 1..6 -> indices 0..5.
        self.anchor_xyz = np.zeros((NUM_ANCHORS, 3))
        for ai in range(NUM_ANCHORS):
            self.anchor_xyz[ai] = np.array(ANCHOR_POSITIONS[FIRST_ANCHOR_ID + ai])
        self.anchor_offsets = np.array(
            [ANCHOR_OFFSETS[FIRST_ANCHOR_ID + ai] for ai in range(NUM_ANCHORS)]
        )

    def on_frame(self, frame: TagFrame):
        st = self.tags.get(frame.tag_id)
        if st is None:
            st = TagState(tag_id=frame.tag_id, kalman=Kalman3D(q=self.kalman_q, r=self.kalman_r))
            self.tags[frame.tag_id] = st

        now = time.monotonic()
        dt = now - st.last_update_t if st.last_update_t > 0 else 0.10
        st.last_update_t = now

        # Build the set of usable anchors: distance > 0 AND fresh
        # (fresh-mask comes from the tag, who only marks anchors that
        # had a successful range this slot AND have valid_count >= 2.)
        valid_idx = []
        valid_dist = []
        for ai in range(NUM_ANCHORS):
            if not frame.is_fresh(ai):
                continue
            d = frame.distances_m[ai]
            if d <= 0.05 or d > 100.0:
                continue
            valid_idx.append(ai)
            valid_dist.append(d + self.anchor_offsets[ai])

        # Always emit the raw anchor distances for diagnostics / GUI parity
        self.sink.send(f"/uwb/tag/{frame.tag_id}/anchors",
                       *[float(d) for d in frame.distances_m])

        if len(valid_idx) < self.min_anchors:
            st.consecutive_misses += 1
            if st.consecutive_misses >= self.LOST_THRESHOLD and not st.is_lost:
                st.is_lost = True
                self.sink.send(f"/uwb/tag/{frame.tag_id}/lost", 1)
            # still predict the kalman forward so velocity decays naturally
            if st.kalman.initialized:
                st.kalman.predict(dt)
                p = st.kalman.position
                self.sink.send(f"/uwb/tag/{frame.tag_id}/pos",
                               float(p[0]), float(p[1]), float(p[2]))
            return

        st.consecutive_misses = 0
        if st.is_lost:
            st.is_lost = False
            self.sink.send(f"/uwb/tag/{frame.tag_id}/lost", 0)

        anchor_pts = self.anchor_xyz[np.array(valid_idx)]
        d_arr = np.array(valid_dist)

        raw = trilaterate(anchor_pts, d_arr)
        if raw is None:
            # geometric collinearity / numerical issue. Predict only.
            if st.kalman.initialized:
                st.kalman.predict(dt)
            return
        st.raw_pos = raw

        st.kalman.predict(dt)
        st.kalman.update(raw)
        smoothed = st.kalman.position

        self.sink.send(f"/uwb/tag/{frame.tag_id}/raw",
                       float(raw[0]), float(raw[1]), float(raw[2]))
        self.sink.send(f"/uwb/tag/{frame.tag_id}/pos",
                       float(smoothed[0]), float(smoothed[1]), float(smoothed[2]))


# ----------------------------------------------------------------------

def run(args):
    sink = OSCSink(host=args.osc_host, port=args.osc_port, enabled=not args.no_osc)
    bridge = Bridge(sink, kalman_q=args.q, kalman_r=args.r,
                    min_anchors=args.min_anchors)

    decoder = FrameDecoder()

    def on_event(ev):
        if isinstance(ev, TagFrame):
            bridge.on_frame(ev)
        elif isinstance(ev, TextLine) and args.verbose:
            print(f"[fw] {ev.text}", file=sys.stderr)

    if args.replay:
        with open(args.replay, "rb") as f:
            data = f.read()
        # Throttle replay to roughly real time so OSC consumers don't drown.
        i = 0
        chunk = 64
        target_byte_rate = 1280  # bytes per second; matches firmware in production
        sleep_per_chunk = chunk / target_byte_rate
        while i < len(data):
            for ev in decoder.feed(data[i:i + chunk]):
                on_event(ev)
            i += chunk
            time.sleep(sleep_per_chunk)
        print("[bridge] replay finished")
        return 0

    # Real serial path
    port = args.port
    if port is None:
        ports = list_ports()
        if not ports:
            print("No serial ports found. Use --port to specify.", file=sys.stderr)
            return 2
        port = ports[0][0]
        print(f"[bridge] auto-selected port {port}")

    try:
        ser = open_serial(port, baud=115200, timeout=0.05)
    except Exception as e:
        print(f"Failed to open {port}: {e}", file=sys.stderr)
        return 2

    reader = SerialReader(ser, on_event=on_event,
                          on_error=lambda e: print(f"[bridge] serial error: {e}", file=sys.stderr))
    reader.start()
    print(f"[bridge] listening on {port}")

    try:
        while True:
            time.sleep(1.0)
            s = reader.stats
            print(f"[bridge] frames={s.good} crc_err={s.bad_crc} bytes={s.bytes_in}",
                  file=sys.stderr)
    except KeyboardInterrupt:
        print("\n[bridge] shutting down")
    finally:
        reader.stop()
        ser.close()

    return 0


def main():
    ap = argparse.ArgumentParser(description="UWB -> trilateration -> synth (OSC) bridge")
    ap.add_argument("--port", help="Serial port for Anchor 1")
    ap.add_argument("--replay", help="Replay a binary capture file at ~real-time")
    ap.add_argument("--osc-host", default="127.0.0.1")
    ap.add_argument("--osc-port", type=int, default=9000)
    ap.add_argument("--no-osc", action="store_true", help="Print JSON to stdout instead of sending OSC")
    ap.add_argument("--q", type=float, default=0.05, help="Kalman process noise (higher=more responsive)")
    ap.add_argument("--r", type=float, default=0.20, help="Kalman measurement noise (higher=more smoothing)")
    ap.add_argument("--min-anchors", type=int, default=4,
                    help="Minimum fresh anchors needed for a 3D fix")
    ap.add_argument("--verbose", action="store_true", help="Print firmware # debug lines")
    args = ap.parse_args()
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
