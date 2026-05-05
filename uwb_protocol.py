"""
uwb_protocol.py
================

Wire format and parser for the Chordinates UWB rig.

Frame layout (little-endian throughout):

    +------+------+------+----------+------+
    | 0xA5 | 0x5A | LEN  | PAYLOAD  | CRC8 |
    +------+------+------+----------+------+
       1      1      1     LEN        1

CRC8 is computed over LEN + PAYLOAD with poly 0x07, init 0x00 (matches
the firmware). Mismatched CRC -> drop the frame, resync at the next
0xA5 0x5A pair.

Payload (LEN = 28):

    type      uint8       0x01 = data
    tag_id    uint8       7..10
    seq       uint8       rolling 0..255 per tag
    fresh     uint8       bitmask: bit i set -> anchor i had a fresh range
    dist[6]   uint16 LE   distance in millimetres (EMA-filtered)
    rssi[6]   int16  LE   signal strength * 100 (dBm * 100)

Anything not framed by the magic bytes is treated as text -- typically
'# ...' debug lines from the firmware. The reader emits these as TextLine
events so a GUI can route them to a debug pane.
"""

from __future__ import annotations

import struct
import threading
from collections import deque
from dataclasses import dataclass, field
from typing import Callable, Iterator, Optional


WIRE_MAGIC = b"\xA5\x5A"
WIRE_TYPE_DATA = 0x01
WIRE_PAYLOAD_LEN = 28
NUM_ANCHORS = 6
NUM_TAGS = 4
FIRST_TAG_ID = 7
FIRST_ANCHOR_ID = 1


def crc8(data: bytes) -> int:
    """CRC-8 with poly 0x07, init 0x00. Matches firmware."""
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


@dataclass
class TagFrame:
    """One decoded data packet from a single tag."""
    tag_id: int
    seq: int
    fresh_mask: int           # bit i = anchor i had a fresh measurement this slot
    distances_m: list[float]  # length NUM_ANCHORS, in metres
    rssi_dbm: list[float]     # length NUM_ANCHORS

    def is_fresh(self, anchor_idx: int) -> bool:
        return bool(self.fresh_mask & (1 << anchor_idx))


@dataclass
class TextLine:
    """A non-binary line from the firmware, e.g. '# [SYNC] beacon'."""
    text: str


@dataclass
class FrameStats:
    """Counters for protocol health monitoring."""
    good: int = 0
    bad_crc: int = 0
    bad_type: int = 0
    bytes_in: int = 0
    text_lines: int = 0


class FrameDecoder:
    """
    Streaming decoder. Feed it bytes via .feed(); it yields a sequence
    of TagFrame and TextLine objects. State is a small ring of bytes
    plus a pending text-line buffer.

    Thread-safety: the decoder itself is NOT thread-safe. Wrap it in a
    lock if you intend to call .feed() from multiple threads.
    """

    def __init__(self) -> None:
        self._buf = bytearray()
        self._text_buf = bytearray()
        self.stats = FrameStats()

    def feed(self, data: bytes) -> Iterator[TagFrame | TextLine]:
        if not data:
            return
        self.stats.bytes_in += len(data)
        self._buf.extend(data)

        while True:
            # Try to find a binary frame anywhere in the buffer.
            magic_idx = self._buf.find(WIRE_MAGIC)

            if magic_idx == -1:
                # No magic in view. Everything in the buffer is text-ish.
                # But keep the last byte in case it's the start of 0xA5 5A.
                if len(self._buf) > 1:
                    yield from self._drain_text(len(self._buf) - 1)
                return

            # Drain everything BEFORE the magic as text first.
            if magic_idx > 0:
                yield from self._drain_text(magic_idx)
                # buffer now starts at the magic.
                magic_idx = 0

            # Need at least magic(2) + len(1) + payload + crc(1).
            if len(self._buf) < 4:
                return  # wait for more bytes

            payload_len = self._buf[2]
            frame_len = 2 + 1 + payload_len + 1

            if frame_len > 256:
                # Bogus length; skip the magic and try again.
                del self._buf[:2]
                self.stats.bad_type += 1
                continue

            if len(self._buf) < frame_len:
                return  # wait for more bytes

            # Verify CRC
            crc_input = bytes(self._buf[2:3 + payload_len])
            expected_crc = self._buf[3 + payload_len]
            if crc8(crc_input) != expected_crc:
                # Corrupt frame. Skip just the first magic byte and resync.
                del self._buf[0]
                self.stats.bad_crc += 1
                continue

            # Frame is valid. Decode.
            payload = bytes(self._buf[3:3 + payload_len])
            decoded = self._decode_payload(payload)
            del self._buf[:frame_len]

            if decoded is not None:
                self.stats.good += 1
                yield decoded
            else:
                self.stats.bad_type += 1

    def _drain_text(self, upto: int) -> Iterator[TextLine]:
        """Treat bytes[0:upto] as text. Split on \\n and yield TextLines."""
        chunk = bytes(self._buf[:upto])
        del self._buf[:upto]

        self._text_buf.extend(chunk)
        while True:
            nl = self._text_buf.find(b"\n")
            if nl < 0:
                break
            line_bytes = bytes(self._text_buf[:nl])
            del self._text_buf[:nl + 1]
            line = line_bytes.decode("utf-8", errors="replace").rstrip("\r")
            if line:  # skip empty lines
                self.stats.text_lines += 1
                yield TextLine(line)

        # cap pending text buffer to avoid unbounded growth on garbage stream
        if len(self._text_buf) > 4096:
            del self._text_buf[:-512]

    def _decode_payload(self, payload: bytes) -> Optional[TagFrame]:
        if len(payload) != WIRE_PAYLOAD_LEN:
            return None
        msg_type = payload[0]
        if msg_type != WIRE_TYPE_DATA:
            return None

        tag_id = payload[1]
        seq = payload[2]
        fresh = payload[3]

        # 6 distances (uint16 mm), 6 rssi (int16 *100)
        dist_mm = struct.unpack_from("<6H", payload, 4)
        rssi_raw = struct.unpack_from("<6h", payload, 16)

        distances_m = [d / 1000.0 for d in dist_mm]
        rssi_dbm = [r / 100.0 for r in rssi_raw]

        return TagFrame(
            tag_id=tag_id,
            seq=seq,
            fresh_mask=fresh,
            distances_m=distances_m,
            rssi_dbm=rssi_dbm,
        )


# -------------------- Threaded reader --------------------

class SerialReader:
    """
    Background thread that pumps bytes from a pyserial Serial object
    through a FrameDecoder and pushes events into a queue/callback.

    Robust to disconnects: catches I/O errors and reports them via
    the on_error callback. Caller decides whether to reconnect.
    """

    def __init__(
        self,
        serial_port,
        on_event: Callable[[TagFrame | TextLine], None],
        on_error: Optional[Callable[[Exception], None]] = None,
        read_size: int = 256,
    ) -> None:
        self._serial = serial_port
        self._on_event = on_event
        self._on_error = on_error
        self._read_size = read_size
        self._stop = threading.Event()
        self._decoder = FrameDecoder()
        self._thread: Optional[threading.Thread] = None

    @property
    def stats(self) -> FrameStats:
        return self._decoder.stats

    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._run, name="UWB-SerialReader", daemon=True)
        self._thread.start()

    def stop(self, join_timeout: float = 1.0) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(join_timeout)
            self._thread = None

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                data = self._serial.read(self._read_size)
            except Exception as e:
                if self._on_error:
                    self._on_error(e)
                return
            if not data:
                continue
            try:
                for event in self._decoder.feed(data):
                    self._on_event(event)
            except Exception as e:
                if self._on_error:
                    self._on_error(e)


# -------------------- Convenience: open a port --------------------

def open_serial(port: str, baud: int = 115200, timeout: float = 0.05):
    """Thin wrapper around pyserial that we import lazily."""
    import serial  # type: ignore
    return serial.Serial(port=port, baudrate=baud, timeout=timeout)


def list_ports() -> list[tuple[str, str]]:
    """Return [(device, description), ...] for connected serial ports."""
    try:
        from serial.tools import list_ports  # type: ignore
    except ImportError:
        return []
    return [(p.device, p.description or "") for p in list_ports.comports()]
