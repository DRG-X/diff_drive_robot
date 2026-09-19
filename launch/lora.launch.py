import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Base-station LoRa receiver.

    Standalone on purpose: it is not included by robot.launch.py or
    mapping.launch.py, so the simulation, EKF and SLAM Toolbox stacks are
    unaffected whether or not this is running.

        ros2 launch diff_drive_robot lora.launch.py fake:=true
        ros2 launch diff_drive_robot lora.launch.py port:=/dev/ttyUSB0
    """
    package_name = 'diff_drive_robot'
    params_path = os.path.join(
        get_package_share_directory(package_name), 'config', 'lora_bridge.yaml')

    declare_port = DeclareLaunchArgument(
        'port', default_value='/dev/ttyUSB0',
        description='Serial device of the base-station LoRa module')

    declare_baudrate = DeclareLaunchArgument(
        'baudrate', default_value='9600',
        description='UART baud rate between the computer and its LoRa module')

    declare_fake = DeclareLaunchArgument(
        'fake', default_value='false',
        description='Synthesise telemetry internally instead of opening the serial port')

    declare_publish_odometry = DeclareLaunchArgument(
        'publish_odometry', default_value='true',
        description='Also republish telemetry as nav_msgs/Odometry on /lora/odom')

    # Deliberately false by default. The rover and its radio run on wall time
    # and have no connection to the simulator's /clock; using sim time here
    # would stamp live telemetry with simulation timestamps, or with zero if
    # no simulator is running.
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use the simulation clock (leave false for real radio hardware)')

    lora_receiver = Node(
        package=package_name,
        executable='lora_receiver',
        name='lora_receiver',
        output='screen',
        parameters=[
            params_path,
            {
                'port': LaunchConfiguration('port'),
                'baudrate': ParameterValue(LaunchConfiguration('baudrate'), value_type=int),
                'fake': ParameterValue(LaunchConfiguration('fake'), value_type=bool),
                'publish_odometry': ParameterValue(
                    LaunchConfiguration('publish_odometry'), value_type=bool),
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool),
            },
        ],
    )

    return LaunchDescription([
        declare_port,
        declare_baudrate,
        declare_fake,
        declare_publish_odometry,
        declare_use_sim_time,
        lora_receiver,
    ])
