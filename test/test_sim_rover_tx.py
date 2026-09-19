"""Tests for the simulation-side transmitter.

Imports rclpy, so it skips itself outside a sourced ROS 2 environment. The
protocol tests cover everything that does not need ROS.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import pytest  # noqa: E402

pytest.importorskip('rclpy', reason='needs a sourced ROS 2 environment')

from lora_bridge.sim_rover_tx_node import yaw_from_quaternion  # noqa: E402


class _Quaternion:
    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
        self.x, self.y, self.z, self.w = x, y, z, w


def _quaternion_from_yaw(yaw):
    return _Quaternion(z=math.sin(yaw * 0.5), w=math.cos(yaw * 0.5))


@pytest.mark.parametrize('yaw', [0.0, 0.1, 1.5707963, -1.5707963, 2.5, -2.5, 3.0, -3.0])
def test_yaw_round_trips_through_a_quaternion(yaw):
    assert yaw_from_quaternion(_quaternion_from_yaw(yaw)) == pytest.approx(yaw, abs=1e-9)


def test_yaw_is_continuous_across_the_pi_boundary():
    # atan2 must wrap rather than jump to a wrong branch, otherwise a robot
    # driving past due-west would report a yaw that flips sign incorrectly.
    just_under = yaw_from_quaternion(_quaternion_from_yaw(3.14159))
    just_over = yaw_from_quaternion(_quaternion_from_yaw(-3.14159))
    assert just_under == pytest.approx(3.14159, abs=1e-6)
    assert just_over == pytest.approx(-3.14159, abs=1e-6)


def test_yaw_ignores_roll_and_pitch_components():
    # The rover reports planar yaw; a tilted IMU must not leak into it.
    roll, pitch, yaw = 0.3, -0.2, 0.9
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    q = _Quaternion(
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy,
        w=cr * cp * cy + sr * sp * sy,
    )
    assert yaw_from_quaternion(q) == pytest.approx(yaw, abs=1e-9)
