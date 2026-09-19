"""Proves the ESP32 firmware and the base station agree on every byte.

The rover runs C and the base station runs Python. If those two encoders ever
disagree the symptom is a silent stream of CRC failures over a radio link with
no debugger attached, so the agreement is checked mechanically here.
"""

import os
import shutil
import subprocess
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import pytest  # noqa: E402

from lora_bridge.protocol import (  # noqa: E402
    MSG_ID_TELEMETRY,
    encode_frame,
    pack_telemetry,
)

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# Must match the `samples` array in firmware/test_parity.c exactly.
SAMPLES = [
    dict(seq=0, rover_time_ms=0, x=0.0, y=0.0, yaw=0.0,
         linear_velocity=0.0, angular_velocity=0.0),
    dict(seq=7, rover_time_ms=1234, x=1.5, y=-2.25, yaw=0.7853981634,
         linear_velocity=0.42, angular_velocity=-0.1),
    dict(seq=1, rover_time_ms=100, x=-0.001, y=0.001, yaw=-3.14159265,
         linear_velocity=0.0, angular_velocity=0.0),
    dict(seq=4294967295, rover_time_ms=4294967295, x=1000.0, y=-1000.0,
         yaw=3.14159265, linear_velocity=1.0, angular_velocity=-1.0),
    dict(seq=42, rover_time_ms=65536, x=0.0, y=12.3456, yaw=1.5707963,
         linear_velocity=1e6, angular_velocity=-1e6),
    dict(seq=255, rover_time_ms=16777216, x=2147.0, y=-2147.0, yaw=0.0001,
         linear_velocity=-0.001, angular_velocity=0.001),
]


@pytest.mark.skipif(shutil.which('cc') is None, reason='no C compiler available')
def test_firmware_frames_match_python_byte_for_byte():
    source = os.path.join(REPO_ROOT, 'firmware', 'test_parity.c')
    with tempfile.TemporaryDirectory() as tmp:
        binary = os.path.join(tmp, 'parity')
        subprocess.run(['cc', '-Wall', '-Wextra', '-Werror', '-O2',
                        '-o', binary, source, '-lm'],
                       check=True, cwd=REPO_ROOT)
        result = subprocess.run([binary], check=True, capture_output=True, text=True)

    firmware_frames = result.stdout.split()
    python_frames = [
        encode_frame(MSG_ID_TELEMETRY, pack_telemetry(**sample)).hex()
        for sample in SAMPLES
    ]

    assert len(firmware_frames) == len(SAMPLES), 'firmware emitted the wrong frame count'
    for index, (from_c, from_python) in enumerate(zip(firmware_frames, python_frames)):
        assert from_c == from_python, (
            'sample %d differs:\n  firmware: %s\n  python:   %s'
            % (index, from_c, from_python))
