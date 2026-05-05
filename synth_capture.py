"""Generate a synthetic byte stream simulating Anchor 1's USB output."""

import math
import os
import struct
import sys

# Make the protocol module importable regardless of cwd
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from uwb_protocol import (
    crc8, WIRE_MAGIC, WIRE_TYPE_DATA, WIRE_PAYLOAD_LEN,
    NUM_ANCHORS, NUM_TAGS, FIRST_TAG_ID,
)


def make_frame(tag_id, seq, fresh_mask, distances_mm, rssi_x100):
    payload = struct.pack("<BBBB", WIRE_TYPE_DATA, tag_id, seq, fresh_mask)
    payload += struct.pack("<6H", *distances_mm)
    payload += struct.pack("<6h", *rssi_x100)
    crc_input = bytes([WIRE_PAYLOAD_LEN]) + payload
    return WIRE_MAGIC + bytes([WIRE_PAYLOAD_LEN]) + payload + bytes([crc8(crc_input)])


def synth(duration_s=10.0, rate_hz=10.0):
    """Each tag drifts on a Lissajous-ish path; produce a real time-ish file."""
    out = bytearray()
    out += b"# UWB Listener Anchor (synthetic)\n"
    out += b"# Ready\n"
    out += b"# [SYNC] beacon\n"

    n_frames = int(duration_s * rate_hz)
    seqs = [0] * NUM_TAGS

    for f in range(n_frames):
        t = f / rate_hz
        for ti in range(NUM_TAGS):
            tag_id = FIRST_TAG_ID + ti
            # tag positions wandering in a 4x4m room
            cx = 2.0 + 1.5 * math.sin(0.3 * t + ti * 1.5)
            cy = 2.0 + 1.5 * math.cos(0.4 * t + ti * 1.0)
            cz = 1.0
            # 6 anchors at the room corners and edges
            anchors = [
                (0.0, 0.0, 0.5), (4.0, 0.0, 0.5),
                (4.0, 4.0, 0.5), (0.0, 4.0, 0.5),
                (2.0, 0.0, 2.5), (2.0, 4.0, 2.5),
            ]
            dists_m = [
                math.sqrt((cx - ax)**2 + (cy - ay)**2 + (cz - az)**2)
                for (ax, ay, az) in anchors
            ]
            dists_mm = [int(min(65535, max(0, d * 1000))) for d in dists_m]

            # rssi: closer = stronger, with floor
            rssi_x100 = [int(-7000 - 200 * d) for d in dists_m]

            # Drop one anchor occasionally to test stale colouring.
            fresh_mask = 0x3F
            if (f + ti) % 25 == 0:
                fresh_mask &= ~(1 << (ti % NUM_ANCHORS))

            out += make_frame(tag_id, seqs[ti], fresh_mask, dists_mm, rssi_x100)
            seqs[ti] = (seqs[ti] + 1) & 0xFF

        if f % 30 == 0:
            out += f"# heartbeat t={t:.1f}s\n".encode()

    return bytes(out)


if __name__ == "__main__":
    out_path = sys.argv[1] if len(sys.argv) > 1 else "synthetic.bin"
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 30.0
    data = synth(duration_s=duration)
    with open(out_path, "wb") as f:
        f.write(data)
    print(f"wrote {len(data)} bytes to {out_path} ({duration}s @ 10Hz x {NUM_TAGS} tags)")
