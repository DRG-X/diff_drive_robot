"""Wire protocol for the rover <-> base LoRa link.

The framing is the COBS variant used by osrf/ros2_serial_example:

    COBS( msg_id | len_hi | len_lo | crc_hi | crc_lo | payload ) 0x00

COBS (Consistent Overhead Byte Stuffing) removes every zero byte from the
encoded frame, which leaves 0x00 free to act as an unambiguous end-of-frame
marker.  A receiver that joins the link mid-stream only has to discard bytes
until the next 0x00 to resynchronise -- important on a radio link where the
first packets after power-up are routinely half-received.

``len`` is the payload length (big endian, header excluded) and ``crc`` is
CRC-16/CCITT-FALSE over the payload only.

This module deliberately imports nothing from ROS so it can be unit tested
with plain python3, and so the ESP32 firmware in ``firmware/`` can be a
line-by-line translation of it.
"""

import struct

# --- message ids -----------------------------------------------------------
# 0x00 is never used: COBS reserves 0x00 as the frame delimiter.
MSG_ID_TELEMETRY = 0x01

HEADER_SIZE = 5
MAX_PAYLOAD_SIZE = 255

# Fixed-point scale factors.  Integers are sent instead of floats because they
# are half the size on the wire and have no endianness/format surprises between
# an ESP32 and an x86 base station.
MM_PER_M = 1000.0        # position:  metres        -> millimetres
CDEG_PER_RAD = 5729.578  # yaw:       radians       -> centidegrees (180/pi*100)
MMS_PER_MS = 1000.0      # linear:    m/s           -> mm/s
MRAD_PER_RAD = 1000.0    # angular:   rad/s         -> mrad/s

TELEMETRY_STRUCT = struct.Struct('<IIiihhh')
TELEMETRY_SIZE = TELEMETRY_STRUCT.size  # 22 bytes


class ProtocolError(Exception):
    """Raised when a frame is malformed. Callers count these and carry on."""


def crc16_ccitt(data, crc=0xFFFF):
    """CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection/xorout)."""
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def cobs_encode(data):
    """COBS-encode ``data``. The result is guaranteed to contain no 0x00."""
    out = bytearray()
    code_index = 0
    out.append(0)  # placeholder, overwritten with the run length below
    code = 1
    for byte in data:
        if byte != 0:
            out.append(byte)
            code += 1
            if code == 0xFF:
                out[code_index] = code
                code_index = len(out)
                out.append(0)
                code = 1
        else:
            out[code_index] = code
            code_index = len(out)
            out.append(0)
            code = 1
    out[code_index] = code
    return bytes(out)


def cobs_decode(data):
    """Inverse of :func:`cobs_encode`. ``data`` must exclude the 0x00 delimiter."""
    out = bytearray()
    index = 0
    length = len(data)
    while index < length:
        code = data[index]
        if code == 0:
            raise ProtocolError('zero byte inside a COBS frame')
        index += 1
        end = index + code - 1
        if end > length:
            raise ProtocolError('COBS frame truncated')
        out += data[index:end]
        index = end
        if code != 0xFF and index < length:
            out.append(0)
    return bytes(out)


def _clamp(value, low, high):
    return low if value < low else (high if value > high else value)


def encode_frame(msg_id, payload):
    """Wrap ``payload`` in a header+CRC and COBS-encode it, delimiter included."""
    if not 0 < msg_id <= 0xFF:
        raise ProtocolError('msg_id must be in 1..255 (0 is the COBS delimiter)')
    if len(payload) > MAX_PAYLOAD_SIZE:
        raise ProtocolError('payload of %d bytes exceeds the %d byte limit'
                            % (len(payload), MAX_PAYLOAD_SIZE))
    crc = crc16_ccitt(payload)
    header = struct.pack('>BHH', msg_id, len(payload), crc)
    return cobs_encode(header + bytes(payload)) + b'\x00'


def decode_frame(frame):
    """Decode one COBS frame body (delimiter already stripped) -> (msg_id, payload)."""
    raw = cobs_decode(frame)
    if len(raw) < HEADER_SIZE:
        raise ProtocolError('frame shorter than the %d byte header' % HEADER_SIZE)
    msg_id, payload_len, crc = struct.unpack('>BHH', raw[:HEADER_SIZE])
    payload = raw[HEADER_SIZE:]
    if len(payload) != payload_len:
        raise ProtocolError('length mismatch: header says %d, got %d'
                            % (payload_len, len(payload)))
    if crc16_ccitt(payload) != crc:
        raise ProtocolError('CRC mismatch')
    return msg_id, payload


def pack_telemetry(seq, rover_time_ms, x, y, yaw, linear_velocity, angular_velocity):
    """Pack rover pose + velocity into the 22-byte telemetry payload.

    Values are clamped rather than allowed to overflow, so a runaway sensor
    reading degrades into a saturated number instead of killing the link.
    """
    return TELEMETRY_STRUCT.pack(
        seq & 0xFFFFFFFF,
        rover_time_ms & 0xFFFFFFFF,
        int(_clamp(round(x * MM_PER_M), -2147483648, 2147483647)),
        int(_clamp(round(y * MM_PER_M), -2147483648, 2147483647)),
        int(_clamp(round(yaw * CDEG_PER_RAD), -32768, 32767)),
        int(_clamp(round(linear_velocity * MMS_PER_MS), -32768, 32767)),
        int(_clamp(round(angular_velocity * MRAD_PER_RAD), -32768, 32767)),
    )


def unpack_telemetry(payload):
    """Inverse of :func:`pack_telemetry` -> dict in SI units."""
    if len(payload) != TELEMETRY_SIZE:
        raise ProtocolError('telemetry payload must be %d bytes, got %d'
                            % (TELEMETRY_SIZE, len(payload)))
    seq, t_ms, x_mm, y_mm, yaw_cdeg, v_mms, w_mrads = TELEMETRY_STRUCT.unpack(payload)
    return {
        'seq': seq,
        'rover_time_ms': t_ms,
        'x': x_mm / MM_PER_M,
        'y': y_mm / MM_PER_M,
        'yaw': yaw_cdeg / CDEG_PER_RAD,
        'linear_velocity': v_mms / MMS_PER_MS,
        'angular_velocity': w_mrads / MRAD_PER_RAD,
    }


class FrameDecoder:
    """Incremental byte-stream -> frame decoder.

    Serial reads arrive in arbitrary chunks, so feed whatever ``read()`` returns
    and iterate the frames that completed.  Corrupt frames are counted and
    dropped; the decoder resynchronises on the next delimiter by itself.
    """

    def __init__(self, max_frame_size=512):
        self._buffer = bytearray()
        self._max_frame_size = max_frame_size
        self.frames_decoded = 0
        self.crc_errors = 0
        self.framing_errors = 0

    def feed(self, data):
        """Append ``data`` and yield every ``(msg_id, payload)`` it completed."""
        self._buffer += data
        frames = []
        while True:
            delimiter = self._buffer.find(0)
            if delimiter < 0:
                # No complete frame yet. Guard against a stuck link filling RAM
                # with a delimiter that never arrives.
                if len(self._buffer) > self._max_frame_size:
                    del self._buffer[:-self._max_frame_size]
                    self.framing_errors += 1
                break
            frame = bytes(self._buffer[:delimiter])
            del self._buffer[:delimiter + 1]
            if not frame:
                continue  # back-to-back delimiters / leading padding
            try:
                msg_id, payload = decode_frame(frame)
            except ProtocolError as exc:
                if 'CRC' in str(exc):
                    self.crc_errors += 1
                else:
                    self.framing_errors += 1
                continue
            self.frames_decoded += 1
            frames.append((msg_id, payload))
        return frames
