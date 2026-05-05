"""Self-test for uwb_protocol.py. Run directly: python test_protocol.py"""

import struct
from uwb_protocol import (
    FrameDecoder, TagFrame, TextLine,
    crc8, WIRE_MAGIC, WIRE_TYPE_DATA, WIRE_PAYLOAD_LEN,
)


def make_frame(tag_id=7, seq=0, fresh_mask=0x3F,
               distances_mm=None, rssi_x100=None) -> bytes:
    """Build a valid binary frame, matching firmware's emitDataPacket()."""
    if distances_mm is None:
        distances_mm = [1000, 1500, 2000, 2500, 3000, 3500]
    if rssi_x100 is None:
        rssi_x100 = [-7500, -8000, -8500, -9000, -9500, -10000]

    payload = struct.pack("<BBBB", WIRE_TYPE_DATA, tag_id, seq, fresh_mask)
    payload += struct.pack("<6H", *distances_mm)
    payload += struct.pack("<6h", *rssi_x100)
    assert len(payload) == WIRE_PAYLOAD_LEN, f"payload len {len(payload)} != {WIRE_PAYLOAD_LEN}"

    crc_input = bytes([WIRE_PAYLOAD_LEN]) + payload
    crc = crc8(crc_input)

    return WIRE_MAGIC + bytes([WIRE_PAYLOAD_LEN]) + payload + bytes([crc])


def test_crc_known_vector():
    # Sanity: CRC of empty is 0.
    assert crc8(b"") == 0
    # Single byte 0x00 -> 0x00
    assert crc8(b"\x00") == 0
    # Single byte 0xFF -> after one shift cycle should be deterministic.
    # We just verify it doesn't crash and gives a value in [0, 255].
    v = crc8(b"\xFF\x01\x02\x03")
    assert 0 <= v <= 255
    print("[OK] CRC basic")


def test_clean_frame():
    dec = FrameDecoder()
    frame = make_frame(tag_id=7, seq=42, fresh_mask=0b00111111)
    events = list(dec.feed(frame))
    assert len(events) == 1, f"expected 1 event, got {len(events)}"
    ev = events[0]
    assert isinstance(ev, TagFrame), type(ev)
    assert ev.tag_id == 7
    assert ev.seq == 42
    assert ev.fresh_mask == 0x3F
    assert ev.distances_m == [1.0, 1.5, 2.0, 2.5, 3.0, 3.5]
    assert all(ev.is_fresh(i) for i in range(6))
    assert dec.stats.good == 1
    assert dec.stats.bad_crc == 0
    print("[OK] clean frame decode")


def test_byte_at_a_time():
    """Ensure decoder works across arbitrary chunk boundaries."""
    dec = FrameDecoder()
    frame = make_frame(tag_id=8, seq=1)
    events = []
    for b in frame:
        events.extend(dec.feed(bytes([b])))
    assert len(events) == 1
    assert events[0].tag_id == 8
    print("[OK] byte-at-a-time")


def test_text_then_frame():
    """Debug text mixed with a binary frame should yield both events in order."""
    dec = FrameDecoder()
    text = b"# hello world\n# more debug\n"
    frame = make_frame(tag_id=9, seq=5)
    events = list(dec.feed(text + frame))
    text_lines = [e for e in events if isinstance(e, TextLine)]
    frames = [e for e in events if isinstance(e, TagFrame)]
    assert len(text_lines) == 2, [t.text for t in text_lines]
    assert text_lines[0].text == "# hello world"
    assert text_lines[1].text == "# more debug"
    assert len(frames) == 1
    assert frames[0].tag_id == 9
    print("[OK] mixed text + frame")


def test_corrupt_frame_recovery():
    """A frame with bad CRC should be discarded; next valid frame still decodes."""
    dec = FrameDecoder()
    bad = bytearray(make_frame(tag_id=7))
    bad[-1] ^= 0xFF  # flip CRC
    good = make_frame(tag_id=10, seq=99)

    events = list(dec.feed(bytes(bad) + good))
    frames = [e for e in events if isinstance(e, TagFrame)]
    assert len(frames) == 1, f"got {len(frames)} frames"
    assert frames[0].tag_id == 10
    assert dec.stats.bad_crc >= 1
    assert dec.stats.good == 1
    print(f"[OK] bad CRC recovery (bad_crc={dec.stats.bad_crc})")


def test_garbage_then_frame():
    """Random bytes that don't look like our magic should be eaten as text."""
    dec = FrameDecoder()
    garbage = bytes([0x12, 0x34, 0x56, 0xA5, 0x00, 0xA5]) + b"\n"  # 0xA5 not followed by 0x5A
    frame = make_frame(tag_id=7, seq=0)
    events = list(dec.feed(garbage + frame))
    frames = [e for e in events if isinstance(e, TagFrame)]
    assert len(frames) == 1
    print("[OK] garbage-before-frame recovery")


def test_truncated_frame_waits():
    """A partial frame shouldn't yield anything; completing it should."""
    dec = FrameDecoder()
    frame = make_frame(tag_id=7)
    events1 = list(dec.feed(frame[:10]))
    assert events1 == [], f"expected nothing, got {events1}"
    events2 = list(dec.feed(frame[10:]))
    frames = [e for e in events2 if isinstance(e, TagFrame)]
    assert len(frames) == 1
    print("[OK] truncated/streamed frame")


def test_multiple_frames_back_to_back():
    dec = FrameDecoder()
    blob = b"".join(make_frame(tag_id=7 + (i % 4), seq=i) for i in range(20))
    events = list(dec.feed(blob))
    frames = [e for e in events if isinstance(e, TagFrame)]
    assert len(frames) == 20, f"expected 20, got {len(frames)}"
    assert frames[0].seq == 0
    assert frames[-1].seq == 19
    print("[OK] 20 back-to-back frames")


if __name__ == "__main__":
    test_crc_known_vector()
    test_clean_frame()
    test_byte_at_a_time()
    test_text_then_frame()
    test_corrupt_frame_recovery()
    test_garbage_then_frame()
    test_truncated_frame_waits()
    test_multiple_frames_back_to_back()
    print("\nAll protocol tests passed.")
