"""
UWB Indoor Position Tracker - Serial Reader with 2D Visualization
Reads JSON distance data from ESP32 DWM3000 tag over USB serial,
computes (x, y) position via trilateration, applies Kalman filtering,
and displays live tracking with distance circles.

Styled after the Processing IDE visualization from the BU03 spatial tracking guide,
adapted for the DWM3000 JSON serial output.

Usage:
    python uwb_serial_tracker.py

Requirements:
    pip install pyserial matplotlib numpy scipy
"""

import serial
import serial.tools.list_ports
import threading
import json
import sys
import numpy as np
from scipy.optimize import least_squares
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle

# ============== CONFIGURATION ==============

# Serial port - update this to match your ESP32
# On Mac:   /dev/cu.usbserial-XXXX
# On Linux: /dev/ttyUSB0
# On Win:   COM3, COM4, etc.
SERIAL_PORT = "auto"  # Set to "auto" to detect, or hardcode your port
BAUD_RATE = 115200

# Anchor positions in METERS (must match your physical setup)
ANCHOR_POSITIONS = {
    "A1": np.array([0.00, 0.30]),    # Up 1 foot (left anchor)
    "A2": np.array([.30, 0.00]),   # Left 3 (from center)
    "A3": np.array([-0.30, 0.00]),    # Right 2 (from center)
}

# Distance calibration offsets in METERS (positive = add, negative = subtract)
# Hold tag at a known distance from each anchor, compare reading, enter correction
ANCHOR_OFFSETS = {
    "A1": 0.0,
    "A2": 0.0,
    "A3": 0.0,
}

# Visualization bounds (in meters)
MIN_X = -1
MAX_X = 1
MIN_Y = -1
MAX_Y = 1

# Anchor display colors
ANCHOR_COLORS = {
    "A1": "#00ff66",
    "A2": "#3399ff",
    "A3": "#ff6600",
}

# Path history length
MAX_PATH_LENGTH = 150

# ============== KALMAN FILTER ==============

class SimpleKalman2D:
    """2D Kalman filter tracking position and velocity."""

    def __init__(self, process_noise=0.12, measurement_noise=1.1, dt=0.10):
        self.state = np.zeros(4)  # [x, y, vx, vy]
        self.P = np.eye(4)
        self.Q = process_noise
        self.R = measurement_noise
        self.dt = dt
        self.initialized = False

    def predict(self):
        if not self.initialized:
            return
        self.state[0] += self.state[2] * self.dt
        self.state[1] += self.state[3] * self.dt
        self.P += self.Q * np.eye(4)

    def update(self, mx, my):
        if not self.initialized:
            self.state[:2] = [mx, my]
            self.state[2:] = [0, 0]
            self.initialized = True
            return mx, my

        Kx = self.P[0, 0] / (self.P[0, 0] + self.R)
        Ky = self.P[1, 1] / (self.P[1, 1] + self.R)

        self.state[0] += Kx * (mx - self.state[0])
        self.state[1] += Ky * (my - self.state[1])

        self.P[0, 0] *= (1 - Kx)
        self.P[1, 1] *= (1 - Ky)

        return self.state[0], self.state[1]

    @property
    def position(self):
        return self.state[0], self.state[1]

# ============== AUTO-DETECT SERIAL PORT ==============

def find_esp32_port():
    ports = serial.tools.list_ports.comports()
    candidates = []
    for p in ports:
        desc = (p.description or "").lower()
        mfg = (p.manufacturer or "").lower()
        if any(kw in desc for kw in ["cp210", "ch340", "ftdi", "usb", "serial", "uart"]):
            candidates.append(p.device)
        elif any(kw in mfg for kw in ["silicon", "wch", "ftdi"]):
            candidates.append(p.device)
    if candidates:
        print(f"[INFO] Detected serial ports: {candidates}")
        return candidates[0]
    if ports:
        print(f"[WARNING] No known ESP32 port. Available: {[p.device for p in ports]}")
        return ports[0].device
    return None

# ============== TRILATERATION ==============

def trilaterate(distances, anchor_positions):
    """Least-squares trilateration from 3+ anchor distances (in meters)."""
    anchors = []
    dists = []
    for key in sorted(distances.keys()):
        if key in anchor_positions and distances[key] > 0:
            anchors.append(anchor_positions[key])
            dists.append(distances[key])

    if len(anchors) < 3:
        return None

    anchors = np.array(anchors)
    dists = np.array(dists)

    def residuals(p):
        return np.sqrt(np.sum((anchors - p) ** 2, axis=1)) - dists

    initial_guess = np.mean(anchors, axis=0)
    try:
        result = least_squares(residuals, initial_guess, method="lm")
        if result.success:
            return result.x
    except Exception as e:
        print(f"[ERROR] Trilateration: {e}")
    return None

# ============== SHARED STATE ==============

latest_distances = {}
latest_rssi = {}
raw_position = None
filtered_position = None
data_lock = threading.Lock()
running = True
kalman = SimpleKalman2D()

# ============== SERIAL READER THREAD ==============

def serial_reader(port, baud):
    global latest_distances, latest_rssi, raw_position, filtered_position, running

    try:
        ser = serial.Serial(port, baud, timeout=1)
        print(f"[INFO] Connected to {port} at {baud} baud")
    except serial.SerialException as e:
        print(f"[ERROR] Could not open {port}: {e}")
        running = False
        return

    buffer = ""
    while running:
        try:
            if ser.in_waiting > 0:
                raw = ser.read(ser.in_waiting).decode("utf-8", errors="ignore")
                buffer += raw

                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    line = line.strip()

                    # Only parse JSON lines (from sendDataOverSerial)
                    if not line.startswith("{"):
                        if line:
                            print(f"  [ESP32] {line}")
                        continue

                    try:
                        json_data = json.loads(line)
                        anchors_data = json_data.get("anchors", {})
                        distances = {}
                        rssi = {}

                        for key, val in anchors_data.items():
                            # DWM3000 firmware sends distance in cm, convert to meters
                            d_cm = float(val.get("distance", 0))
                            d_m = d_cm / 100.0 + ANCHOR_OFFSETS.get(key, 0.0)
                            distances[key] = d_m
                            rssi[key] = float(val.get("rssi", 0))

                        pos = trilaterate(distances, ANCHOR_POSITIONS)

                        with data_lock:
                            latest_distances = distances.copy()
                            latest_rssi = rssi.copy()

                            if pos is not None:
                                raw_position = pos.copy()
                                kalman.predict()
                                fx, fy = kalman.update(pos[0], pos[1])
                                filtered_position = np.array([fx, fy])
                            else:
                                kalman.predict()
                                fx, fy = kalman.position
                                filtered_position = np.array([fx, fy])

                        dist_str = " | ".join(f"{k}: {v:.2f}m" for k, v in sorted(distances.items()))
                        pos_str = f"({pos[0]:.2f}, {pos[1]:.2f})" if pos is not None else "N/A"
                        print(f"[DATA] {dist_str}  ->  Pos: {pos_str}")

                    except (json.JSONDecodeError, KeyError, ValueError) as e:
                        print(f"[WARN] Parse error: {e}")

        except serial.SerialException:
            print("[ERROR] Serial connection lost")
            break
        except Exception as e:
            print(f"[ERROR] {e}")

    ser.close()
    print("[INFO] Serial reader stopped")

# ============== MATPLOTLIB VISUALIZATION ==============

def run_visualization():
    global running

    fig, ax = plt.subplots(figsize=(10, 12))
    fig.patch.set_facecolor("#0a0a0a")
    ax.set_facecolor("#111111")

    # --- Grid (0.5m spacing) ---
    for x in np.arange(np.floor(MIN_X), MAX_X + 0.5, 0.5):
        ax.axvline(x, color="#1a1a1a", linewidth=0.5)
    for y in np.arange(np.floor(MIN_Y), MAX_Y + 0.5, 0.5):
        ax.axhline(y, color="#1a1a1a", linewidth=0.5)

    # Stronger lines at origin
    ax.axvline(0, color="#333333", linewidth=1.5)
    ax.axhline(0, color="#333333", linewidth=1.5)

    # --- Static anchor markers ---
    for key, pos in ANCHOR_POSITIONS.items():
        color = ANCHOR_COLORS.get(key, "#ffffff")
        ax.plot(pos[0], pos[1], "s", color=color, markersize=12, zorder=10)
        ax.annotate(
            key, (pos[0], pos[1]),
            textcoords="offset points", xytext=(0, 14),
            ha="center", fontsize=11, fontweight="bold", color=color
        )

    # --- Dynamic: distance circles + distance labels ---
    circle_artists = {}
    dist_text_artists = {}
    for key in ANCHOR_POSITIONS:
        color = ANCHOR_COLORS.get(key, "#ffffff")
        circ = Circle(
            ANCHOR_POSITIONS[key], 0.001,
            fill=True, facecolor=color + "0d",
            edgecolor=color, linewidth=1.2, zorder=2
        )
        ax.add_patch(circ)
        circle_artists[key] = circ

        txt = ax.text(
            ANCHOR_POSITIONS[key][0], ANCHOR_POSITIONS[key][1] - 0.22, "",
            color=color, ha="center", va="top", fontsize=9, zorder=11
        )
        dist_text_artists[key] = txt

    # --- Dynamic: raw position (red) ---
    (raw_dot,) = ax.plot([], [], "o", color="#ff6666", markersize=10, zorder=15)
    (raw_ch,) = ax.plot([], [], "+", color="#ff6666", markersize=14, markeredgewidth=2, zorder=14)

    # --- Dynamic: filtered position (green) ---
    (filt_dot,) = ax.plot([], [], "o", color="#66ff88", markersize=10, zorder=16)
    (filt_ch,) = ax.plot([], [], "+", color="#66ff88", markersize=14, markeredgewidth=2, zorder=14)

    # --- Dynamic: filtered path trail ---
    (path_line,) = ax.plot([], [], "-", color="#66ff88", alpha=0.35, linewidth=1.2, zorder=3)
    path_x, path_y = [], []

    # --- Info text at top ---
    info_text = ax.text(
        (MIN_X + MAX_X) / 2, MAX_Y + 0.2, "Waiting for data...",
        ha="center", va="bottom", fontsize=11, color="#ffffff", fontweight="bold"
    )

    # --- Legend (top-right corner) ---
    legend_items = [
        ("A1", "s", ANCHOR_COLORS["A1"], "Base A1"),
        ("A2", "s", ANCHOR_COLORS["A2"], "Base A2"),
        ("A3", "s", ANCHOR_COLORS["A3"], "Base A3"),
        ("raw", "o", "#ff6666", "Raw Position"),
        ("filt", "o", "#66ff88", "Filtered Position"),
    ]
    lx = MAX_X - 0.1
    ly = MAX_Y - 0.15
    for i, (_, marker, color, label) in enumerate(legend_items):
        yy = ly - i * 0.3
        ax.plot(lx, yy, marker, color=color, markersize=6, zorder=20, clip_on=False)
        ax.text(lx + 0.12, yy, label, color="#bbbbbb", fontsize=8, va="center", zorder=20, clip_on=False)

    # --- Axes ---
    ax.set_xlim(MIN_X, MAX_X)
    ax.set_ylim(MIN_Y, MAX_Y)
    ax.set_aspect("equal")
    ax.set_title("UWB Indoor Position Tracker", fontsize=14, color="#ffffff", fontweight="bold", pad=15)
    ax.set_xlabel("X (meters)", color="#aaaaaa", fontsize=10, labelpad=8)
    ax.set_ylabel("Y (meters)", color="#aaaaaa", fontsize=10, labelpad=8)
    ax.tick_params(colors="#555555", labelsize=9)
    for spine in ax.spines.values():
        spine.set_color("#2a2a2a")

    # --- Animation update ---
    def update(frame):
        nonlocal path_x, path_y

        with data_lock:
            dists = latest_distances.copy()
            rpos = raw_position.copy() if raw_position is not None else None
            fpos = filtered_position.copy() if filtered_position is not None else None

        # Distance circles and labels
        for key in ANCHOR_POSITIONS:
            d = dists.get(key, 0)
            circle_artists[key].set_radius(d if d > 0 else 0.001)
            dist_text_artists[key].set_text(f"{d:.2f}m" if d > 0 else "")

        # Raw position
        if rpos is not None:
            raw_dot.set_data([rpos[0]], [rpos[1]])
            raw_ch.set_data([rpos[0]], [rpos[1]])
        else:
            raw_dot.set_data([], [])
            raw_ch.set_data([], [])

        # Filtered position
        if fpos is not None:
            filt_dot.set_data([fpos[0]], [fpos[1]])
            filt_ch.set_data([fpos[0]], [fpos[1]])

            path_x.append(fpos[0])
            path_y.append(fpos[1])
            if len(path_x) > MAX_PATH_LENGTH:
                path_x.pop(0)
                path_y.pop(0)
            path_line.set_data(path_x, path_y)

            parts = []
            if rpos is not None:
                parts.append(f"RAW ({rpos[0]:.2f}, {rpos[1]:.2f})")
            parts.append(f"FILTERED ({fpos[0]:.2f}, {fpos[1]:.2f})")
            info_text.set_text("   |   ".join(parts))
        else:
            filt_dot.set_data([], [])
            filt_ch.set_data([], [])
            info_text.set_text("Waiting for data...")

        return (
            raw_dot, raw_ch, filt_dot, filt_ch,
            path_line, info_text,
            *circle_artists.values(),
            *dist_text_artists.values(),
        )

    ani = animation.FuncAnimation(fig, update, interval=100, cache_frame_data=False, blit=False)

    try:
        plt.tight_layout()
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        running = False
        plt.close()

# ============== MAIN ==============

if __name__ == "__main__":
    port = SERIAL_PORT
    if port == "auto":
        port = find_esp32_port()
        if port is None:
            print("[ERROR] No serial port found. Plug in your ESP32 and try again.")
            print("        Or set SERIAL_PORT manually in this script.")
            sys.exit(1)
    print(f"[INFO] Using serial port: {port}")

    reader_thread = threading.Thread(target=serial_reader, args=(port, BAUD_RATE), daemon=True)
    reader_thread.start()

    run_visualization()
    print("[INFO] Done.")