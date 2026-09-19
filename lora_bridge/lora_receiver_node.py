#!/usr/bin/env python3
"""Base-station node: LoRa module UART -> ROS 2 topics.

Sits at the end of the chain::

    rover: sensors -> ESP32 -> UART -> LoRa module ~~RF~~>
    base:  LoRa module -> UART/USB-TTL -> this node -> ROS 2

Everything it publishes lives under the ``/lora`` namespace, and it subscribes
to nothing and broadcasts no TF, so it cannot perturb /cmd_vel, the LiDAR, the
EKF or SLAM Toolbox.

Set ``fake:=true`` to run the entire decode path against an internally
generated stream and no serial port at all.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

from diff_drive_robot.msg import RoverTelemetry

from lora_bridge.protocol import (
    FrameDecoder,
    MSG_ID_TELEMETRY,
    ProtocolError,
    encode_frame,
    pack_telemetry,
    unpack_telemetry,
)
from lora_bridge.rover_sim import SimulatedRover


class LoraReceiver(Node):

    def __init__(self):
        super().__init__('lora_receiver')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('fake', False)
        self.declare_parameter('poll_rate', 50.0)
        self.declare_parameter('fake_telemetry_rate', 2.0)
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'lora_rover')
        self.declare_parameter('publish_odometry', True)
        self.declare_parameter('link_timeout', 5.0)
        self.declare_parameter('reconnect_period', 2.0)

        self.port = self.get_parameter('port').value
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.fake = bool(self.get_parameter('fake').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        self.link_timeout = float(self.get_parameter('link_timeout').value)
        self.reconnect_period = float(self.get_parameter('reconnect_period').value)

        # Telemetry over a radio link is inherently lossy, and a late packet is
        # worthless -- best effort with a shallow queue matches the medium.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.telemetry_pub = self.create_publisher(RoverTelemetry, 'lora/telemetry', qos)
        self.link_pub = self.create_publisher(Bool, 'lora/link_ok', qos)
        self.odom_pub = None
        if bool(self.get_parameter('publish_odometry').value):
            self.odom_pub = self.create_publisher(Odometry, 'lora/odom', qos)

        self.decoder = FrameDecoder()
        self.serial = None
        self.last_frame_time = None
        self.link_ok = False
        self.last_seq = None
        self.packets_lost = 0

        if self.fake:
            self.sim = SimulatedRover()
            rate = float(self.get_parameter('fake_telemetry_rate').value)
            self.fake_period = 1.0 / rate
            self.create_timer(self.fake_period, self._generate_fake_bytes)
            self.get_logger().info(
                'fake mode: synthesising telemetry at %.1f Hz, no serial port opened' % rate)
        else:
            self._open_serial()
            poll_rate = float(self.get_parameter('poll_rate').value)
            self.create_timer(1.0 / poll_rate, self._read_serial)
            self.create_timer(self.reconnect_period, self._ensure_serial)

        self.create_timer(1.0, self._check_link)

    # -- transport ----------------------------------------------------------

    def _open_serial(self):
        try:
            import serial
        except ImportError:
            self.get_logger().fatal(
                'pyserial is not installed. Run: sudo apt install python3-serial')
            raise
        try:
            # timeout=0 makes read() non-blocking, so the executor is never
            # stalled waiting on a radio that has gone quiet.
            self.serial = serial.Serial(self.port, self.baudrate, timeout=0)
            self.get_logger().info('opened %s at %d baud' % (self.port, self.baudrate))
        except Exception as exc:  # serial.SerialException and friends
            self.serial = None
            self.get_logger().warn('could not open %s: %s (retrying)' % (self.port, exc))

    def _ensure_serial(self):
        if self.serial is None:
            self._open_serial()

    def _read_serial(self):
        if self.serial is None:
            return
        try:
            data = self.serial.read(4096)
        except Exception as exc:
            self.get_logger().warn('serial read failed: %s (reopening)' % exc)
            try:
                self.serial.close()
            except Exception:
                pass
            self.serial = None
            return
        if data:
            self._consume(data)

    def _generate_fake_bytes(self):
        """Encode a real frame from the simulated rover and feed it back in.

        The frame is split in two so the incremental decoder's partial-frame
        path gets exercised the same way a chunked serial read would.
        """
        state = self.sim.step(self.fake_period)
        frame = encode_frame(MSG_ID_TELEMETRY, pack_telemetry(**state))
        midpoint = len(frame) // 2
        self._consume(frame[:midpoint])
        self._consume(frame[midpoint:])

    # -- decoding -----------------------------------------------------------

    def _consume(self, data):
        for msg_id, payload in self.decoder.feed(data):
            if msg_id != MSG_ID_TELEMETRY:
                self.get_logger().warn('ignoring unknown msg_id 0x%02x' % msg_id, once=True)
                continue
            try:
                state = unpack_telemetry(payload)
            except ProtocolError as exc:
                self.get_logger().warn('bad telemetry payload: %s' % exc)
                continue
            self._publish(state)

    def _publish(self, state):
        now = self.get_clock().now()
        self.last_frame_time = now

        if self.last_seq is not None:
            gap = (state['seq'] - self.last_seq) & 0xFFFFFFFF
            if 1 < gap < 1000:  # ignore a rover reboot, which resets seq to 0
                self.packets_lost += gap - 1
        self.last_seq = state['seq']

        msg = RoverTelemetry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.frame_id
        msg.seq = state['seq']
        msg.rover_time_ms = state['rover_time_ms']
        msg.x = state['x']
        msg.y = state['y']
        msg.yaw = state['yaw']
        msg.linear_velocity = state['linear_velocity']
        msg.angular_velocity = state['angular_velocity']
        msg.frames_received = self.decoder.frames_decoded
        msg.packets_lost = self.packets_lost
        msg.crc_errors = self.decoder.crc_errors
        self.telemetry_pub.publish(msg)

        if self.odom_pub is not None:
            self.odom_pub.publish(self._to_odometry(msg))

        if not self.link_ok:
            self.link_ok = True
            self.link_pub.publish(Bool(data=True))
            self.get_logger().info('link up (seq %d)' % state['seq'])

    def _to_odometry(self, telemetry):
        """Build a nav_msgs/Odometry view of the telemetry for RViz.

        Covariances are left at zero: this is a monitoring topic, it is not
        intended as an input to the EKF.
        """
        odom = Odometry()
        odom.header = telemetry.header
        odom.child_frame_id = self.child_frame_id
        odom.pose.pose.position.x = telemetry.x
        odom.pose.pose.position.y = telemetry.y
        odom.pose.pose.orientation.z = math.sin(telemetry.yaw * 0.5)
        odom.pose.pose.orientation.w = math.cos(telemetry.yaw * 0.5)
        odom.twist.twist.linear.x = telemetry.linear_velocity
        odom.twist.twist.angular.z = telemetry.angular_velocity
        return odom

    # -- link watchdog ------------------------------------------------------

    def _check_link(self):
        """Publish link state once a second, whether or not it changed.

        A steady heartbeat rather than edge-triggered updates: with best-effort
        QoS a subscriber that starts late would otherwise see nothing at all,
        and "no messages" is indistinguishable from "node not running".
        """
        if self.last_frame_time is not None:
            age = (self.get_clock().now() - self.last_frame_time).nanoseconds / 1e9
            if self.link_ok and age > self.link_timeout:
                self.link_ok = False
                self.get_logger().warn('link down: no telemetry for %.1f s' % age)
        self.link_pub.publish(Bool(data=self.link_ok))

    def destroy_node(self):
        if self.serial is not None:
            try:
                self.serial.close()
            except Exception:
                pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LoraReceiver()
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
