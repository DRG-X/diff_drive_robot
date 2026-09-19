"""A tiny differential-drive model used to produce telemetry without hardware.

This exists so the whole LoRa path -- pack, CRC, COBS, serial framing, decode,
publish -- can be exercised before an ESP32 or a radio is on the desk.  It is
not connected to Gazebo; it is a stand-in for the *real rover*, which in the
final system is an independent machine far away from the ROS 2 graph.
"""

import math


class SimulatedRover:
    """Drives a gentle S-curve so the reported pose visibly changes."""

    def __init__(self, linear_speed=0.25, angular_amplitude=0.4, angular_period=20.0):
        self.linear_speed = linear_speed
        self.angular_amplitude = angular_amplitude
        self.angular_period = angular_period
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.elapsed = 0.0
        self.seq = 0

    def step(self, dt):
        """Advance the model by ``dt`` seconds and return the new state."""
        self.elapsed += dt
        angular_velocity = self.angular_amplitude * math.sin(
            2.0 * math.pi * self.elapsed / self.angular_period)
        linear_velocity = self.linear_speed

        self.yaw += angular_velocity * dt
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))  # wrap to +-pi
        self.x += linear_velocity * math.cos(self.yaw) * dt
        self.y += linear_velocity * math.sin(self.yaw) * dt
        self.seq += 1

        return {
            'seq': self.seq,
            'rover_time_ms': int(self.elapsed * 1000.0),
            'x': self.x,
            'y': self.y,
            'yaw': self.yaw,
            'linear_velocity': linear_velocity,
            'angular_velocity': angular_velocity,
        }
