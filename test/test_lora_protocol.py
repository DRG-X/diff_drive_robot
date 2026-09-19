"""Protocol tests. Pure python, no ROS or hardware required.

    python3 -m pytest test/test_lora_protocol.py
"""

import os
import random
import sys
import tty

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import pytest  # noqa: E402

from lora_bridge.protocol import (  # noqa: E402
    FrameDecoder,
    MSG_ID_TELEMETRY,
    ProtocolError,
    cobs_decode,
    cobs_encode,
    crc16_ccitt,
    decode_frame,
    encode_frame,
    pack_telemetry,
    unpack_telemetry,
)
from lora_bridge.rover_sim import SimulatedRover  # noqa: E402

SAMPLE = dict(seq=7, rover_time_ms=1234, x=1.5, y=-2.25, yaw=0.7853981634,
              linear_velocity=0.42, angular_velocity=-0.1)


def test_crc16_ccitt_reference_vector():
    # The canonical CRC-16/CCITT-FALSE check value for b"123456789".
    assert crc16_ccitt(b'123456789') == 0x29B1


@pytest.mark.parametrize('payload', [
    b'', b'\x00', b'\x00\x00', b'\x01\x02\x03', b'\x00' * 300,
    bytes(range(256)), b'\x01' * 254, b'\x01' * 255, b'\x01' * 256,
])
def test_cobs_round_trip(payload):
    encoded = cobs_encode(payload)
    assert 0 not in encoded, 'COBS output must be free of the frame delimiter'
    assert cobs_decode(encoded) == payload


def test_frame_round_trip():
    frame = encode_frame(MSG_ID_TELEMETRY, pack_telemetry(**SAMPLE))
    assert frame.endswith(b'\x00')
    assert 0 not in frame[:-1]
    msg_id, payload = decode_frame(frame[:-1])
    assert msg_id == MSG_ID_TELEMETRY
    decoded = unpack_telemetry(payload)
    assert decoded['seq'] == SAMPLE['seq']
    assert decoded['rover_time_ms'] == SAMPLE['rover_time_ms']
    for key in ('x', 'y', 'linear_velocity', 'angular_velocity'):
        assert decoded[key] == pytest.approx(SAMPLE[key], abs=1e-3)
    assert decoded['yaw'] == pytest.approx(SAMPLE['yaw'], abs=2e-4)


def test_frame_size_fits_a_lora_packet():
    frame = encode_frame(MSG_ID_TELEMETRY, pack_telemetry(**SAMPLE))
    assert len(frame) <= 64, 'telemetry frame must stay well inside one LoRa packet'


def test_telemetry_values_are_clamped_not_wrapped():
    # A runaway reading must saturate rather than wrap to a plausible-looking
    # value, which would silently corrupt the pose shown at the base station.
    decoded = unpack_telemetry(pack_telemetry(seq=0, rover_time_ms=0, x=0.0, y=0.0,
                                              yaw=0.0, linear_velocity=1e6,
                                              angular_velocity=-1e6))
    assert decoded['linear_velocity'] == pytest.approx(32.767)
    assert decoded['angular_velocity'] == pytest.approx(-32.768)


def test_crc_error_is_detected_and_counted():
    frame = bytearray(encode_frame(MSG_ID_TELEMETRY, pack_telemetry(**SAMPLE)))
    frame[8] ^= 0xFF  # flip bits inside the COBS-encoded payload
    decoder = FrameDecoder()
    assert decoder.feed(bytes(frame)) == []
    assert decoder.crc_errors + decoder.framing_errors == 1
    assert decoder.frames_decoded == 0


def test_decoder_reassembles_frames_split_across_reads():
    frames = [encode_frame(MSG_ID_TELEMETRY, pack_telemetry(**dict(SAMPLE, seq=i)))
              for i in range(20)]
    stream = b''.join(frames)
    decoder = FrameDecoder()
    received = []
    index = 0
    rng = random.Random(1234)
    while index < len(stream):
        chunk = rng.randint(1, 13)
        received += decoder.feed(stream[index:index + chunk])
        index += chunk
    assert decoder.frames_decoded == 20
    assert [unpack_telemetry(p)['seq'] for _, p in received] == list(range(20))


def test_decoder_resynchronises_after_garbage():
    # Simulate joining the radio stream mid-packet: leading junk, then a good
    # frame. The junk must be discarded and the good frame still recovered.
    good = encode_frame(MSG_ID_TELEMETRY, pack_telemetry(**SAMPLE))
    decoder = FrameDecoder()
    frames = decoder.feed(b'\x7f\x3a\xd1' + good[len(good) // 2:] + good)
    assert len(frames) == 1
    assert unpack_telemetry(frames[0][1])['seq'] == SAMPLE['seq']


def test_decoder_survives_a_delimiter_that_never_arrives():
    decoder = FrameDecoder(max_frame_size=64)
    for _ in range(100):
        decoder.feed(b'\x42' * 64)
    assert len(decoder._buffer) <= 64, 'buffer must not grow without bound'


def test_msg_id_zero_is_rejected():
    # 0x00 is the frame delimiter and can never be a valid message id.
    with pytest.raises(ProtocolError):
        encode_frame(0, b'abc')


def test_simulated_rover_advances_and_wraps_yaw():
    rover = SimulatedRover()
    for _ in range(2000):
        state = rover.step(0.1)
        assert -3.1416 <= state['yaw'] <= 3.1416
    assert state['seq'] == 2000
    assert abs(state['x']) + abs(state['y']) > 0.1, 'rover should have moved'


def test_round_trip_over_a_real_serial_pty():
    """End-to-end over an actual OS serial device, no pyserial needed."""
    controller, peripheral = os.openpty()
    # Raw mode is mandatory for binary framing: a pty defaults to canonical
    # mode, where reads block until a newline and ONLCR rewrites every 0x0A
    # into 0x0D 0x0A. That is exactly why the socat recipe in the docs says
    # `raw,echo=0` -- get it wrong on real hardware and frames arrive mangled.
    tty.setraw(controller)
    tty.setraw(peripheral)
    try:
        expected = []
        rover = SimulatedRover()
        for _ in range(5):
            state = rover.step(0.5)
            expected.append(state['seq'])
            os.write(controller, encode_frame(MSG_ID_TELEMETRY, pack_telemetry(**state)))

        decoder = FrameDecoder()
        received = []
        while len(received) < 5:
            received += decoder.feed(os.read(peripheral, 64))
        assert [unpack_telemetry(p)['seq'] for _, p in received] == expected
    finally:
        os.close(controller)
        os.close(peripheral)
