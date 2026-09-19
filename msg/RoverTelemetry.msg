# Rover pose and velocity as received over the LoRa uplink.
#
# Published by the lora_receiver node on /lora/telemetry. Pose is expressed in
# the rover's own odometry frame, which is independent of the base station's
# ROS 2 TF tree.

std_msgs/Header header

# Rover-side packet counter, wraps at 2^32. Used to detect dropped packets.
uint32 seq
# Rover uptime in milliseconds when the packet was assembled.
uint32 rover_time_ms

# Pose: metres and radians. Transmitted as mm / centidegrees, see protocol.py.
float64 x
float64 y
float64 yaw

# Body velocity: m/s and rad/s.
float64 linear_velocity
float64 angular_velocity

# Link health, counted at the base station. Not transmitted over the air.
uint32 frames_received
uint32 packets_lost
uint32 crc_errors
