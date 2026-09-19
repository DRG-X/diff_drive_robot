#!/usr/bin/env python3
"""ESP32 stand-in that transmits the simulated robot's real pose.

``fake_rover_tx`` invents its own motion, which is fine for proving the
decoder works but means the telemetry has nothing to do with the robot in
Gazebo. This node closes that gap: it subscribes to the simulator's odometry
and emits the same LoRa frames the firmware would, so the full path can be
driven from the keyboard::

    Gazebo -> /odom -> sim_rover_tx -> pty --(socat)--> pty -> lora_receiver
                                                                    |
                                                          /lora/telemetry

Pair it with socat, exactly as you would pair the real ESP32 with a radio:

    socat -d -d pty,raw,echo=0,link=/tmp/ttyROVER pty,raw,echo=0,link=/tmp/ttyBASE

This is a test fixture, not part of the rover. On real hardware the ESP32 does
this job and this node does not run.
"""

import math
import random

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry

from lora_bridge.protocol import MSG_ID_TELEMETRY, encode_frame, pack_telemetry


def yaw_from_quaternion(q):
    """Extract the yaw angle from a geometry_msgs Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class SimRoverTx(Node):

    def __init__(self):
        super().__init__('sim_rover_tx')

        self.declare_parameter('port', '/tmp/ttyROVER')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('rate', 2.0)
        self.declare_parameter('drop_probability', 0.0)

        self.port = self.get_parameter('port').value
        self.baudrate = int(self.get_parameter('baudrate').value)
        odom_topic = self.get_parameter('odom_topic').value
        rate = float(self.get_parameter('rate').value)

        # Lets the base station's watchdog and lost-packet counter be exercised
        # in simulation, where the link is otherwise perfect and those paths
        # would never run until the radio is out in a field.
        self.drop_probability = float(self.get_parameter('drop_probability').value)

        self.serial = None
        self.latest_odom = None
        self.seq = 0
        self.sent = 0
        self.dropped = 0
        self.start_time = self.get_clock().now()

        self._open_serial()

        # The simulator publishes odometry far faster than any LoRa link can
        # carry it, so take the latest sample on a timer rather than
        # transmitting every message.
        self.create_subscription(
            Odometry, odom_topic, self._on_odom,
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST, depth=1))
        self.create_timer(1.0 / rate, self._transmit)

        self.get_logger().info(
            'relaying %s to %s at %.1f Hz (drop probability %.2f)'
            % (odom_topic, self.port, rate, self.drop_probability))

    def _open_serial(self):
        try:
            import serial
        except ImportError:
            self.get_logger().fatal(
                'pyserial is not installed. Run: sudo apt install python3-serial')
            raise
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=1)
        except Exception as exc:
            self.get_logger().fatal(
                'could not open %s: %s\n'
                'Start the virtual serial pair first:\n'
                '  socat -d -d pty,raw,echo=0,link=/tmp/ttyROVER '
                'pty,raw,echo=0,link=/tmp/ttyBASE' % (self.port, exc))
            raise

    def _on_odom(self, msg):
        self.latest_odom = msg

    def _transmit(self):
        if self.latest_odom is None:
            self.get_logger().warn('no odometry received yet', once=True)
            return

        odom = self.latest_odom
        uptime_ms = int(
            (self.get_clock().now() - self.start_time).nanoseconds / 1e6)

        payload = pack_telemetry(
            seq=self.seq,
            rover_time_ms=uptime_ms,
            x=odom.pose.pose.position.x,
            y=odom.pose.pose.position.y,
            yaw=yaw_from_quaternion(odom.pose.pose.orientation),
            linear_velocity=odom.twist.twist.linear.x,
            angular_velocity=odom.twist.twist.angular.z,
        )
        self.seq += 1

        # The sequence number still advances on a dropped packet -- that is
        # what makes the gap visible to the receiver, exactly as a frame lost
        # to interference would be.
        if self.drop_probability > 0.0 and random.random() < self.drop_probability:
            self.dropped += 1
            return

        try:
            self.serial.write(encode_frame(MSG_ID_TELEMETRY, payload))
            self.serial.flush()
        except Exception as exc:
            self.get_logger().warn('serial write failed: %s' % exc)
            return

        self.sent += 1
        if self.sent % 20 == 0:
            self.get_logger().info(
                'sent %d, dropped %d, x=%.2f y=%.2f'
                % (self.sent, self.dropped, odom.pose.pose.position.x,
                   odom.pose.pose.position.y))

    def destroy_node(self):
        if self.serial is not None:
            try:
                self.serial.close()
            except Exception:
                pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SimRoverTx()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
