# LoRa telemetry bridge

Long-range, low-bandwidth uplink that lets the rover report where it is to a
computer far outside Wi-Fi range.

```
  ROVER                                          BASE STATION
  ┌──────────────┐                               ┌──────────────┐
  │ sensors      │                               │ LoRa module  │
  │   ↓          │                               │   ↓ UART     │
  │ ESP32        │                               │ USB-TTL      │
  │   ↓ UART     │      ~~~~ RF, 433/868/915 ~~~>│   ↓          │
  │ LoRa module  │                               │ lora_receiver│
  └──────────────┘                               │   ↓          │
                                                 │ ROS 2 topics │
                                                 └──────────────┘
```

UART is only ever a local wire: ESP32 ↔ its LoRa module, and computer ↔ its
LoRa module. The hop between them is radio.

## What it publishes

| Topic | Type | Notes |
|---|---|---|
| `/lora/telemetry` | `diff_drive_robot/msg/RoverTelemetry` | Pose, velocity, link counters |
| `/lora/odom` | `nav_msgs/msg/Odometry` | Same data for RViz; optional |
| `/lora/link_ok` | `std_msgs/msg/Bool` | False after `link_timeout` with no frames |

All QoS is **best effort** — a late telemetry packet is worthless, and a
reliable queue over a lossy radio only builds backlog.

### This does not touch the existing stack

The receiver subscribes to nothing and broadcasts no TF. Everything it
publishes is namespaced under `/lora`. `/cmd_vel`, `/scan`, `/odom`, `/imu`,
the EKF and SLAM Toolbox are untouched whether or not it is running, and
`lora.launch.py` is standalone — neither `robot.launch.py` nor
`mapping.launch.py` includes it.

`/lora/odom` is a **monitoring** topic. Its covariances are zero and its
`child_frame_id` is `lora_rover`, deliberately not `base_link`. Do not add it
to `ekf.yaml`.

## Wire protocol

Framing is the COBS variant from [`osrf/ros2_serial_example`][ros2_serial]:

```
COBS( msg_id | len_hi | len_lo | crc_hi | crc_lo | payload ) 0x00
```

COBS strips every zero byte from the encoded frame, which leaves `0x00` free
as an unambiguous end-of-frame marker. A receiver joining the stream mid-packet
just discards bytes until the next `0x00` and it is back in sync — which
happens constantly on a real radio link.

Header fields are big endian; `len` excludes the header; `crc` is
CRC-16/CCITT-FALSE over the payload only.

`msg_id 0x01` is telemetry, a 22-byte little-endian payload:

| Field | Type | Scale |
|---|---|---|
| `seq` | `uint32` | — |
| `rover_time_ms` | `uint32` | rover uptime, ms |
| `x`, `y` | `int32` | millimetres |
| `yaw` | `int16` | centidegrees |
| `linear_velocity` | `int16` | mm/s |
| `angular_velocity` | `int16` | mrad/s |

Total frame: **29 bytes**. At 2 Hz that is ~58 B/s, comfortably inside LoRa's
throughput at SF7–SF9 and inside the 1% duty-cycle limits that apply in some
regions. Fixed-point integers are used rather than floats because they are
half the size and have no format ambiguity between an ESP32 and an x86 host.
Out-of-range values saturate instead of wrapping, so a bad sensor reading
shows up as an obviously pinned number rather than a plausible wrong one.

The `msg_id` byte costs nothing now and means a second message type can be
added later without touching the framing.

> **Note on the reference.** `osrf/ros2_serial_example` is from 2019 and
> targets ROS 2 Crystal; it will not build on Lyrical. What is reused here is
> its framing design, not its code. The transport underneath is plain
> `pyserial`.

[ros2_serial]: https://github.com/osrf/ros2_serial_example

## Build

```bash
sudo apt install ros-lyrical-rclpy python3-serial
cd ~/ros2_ws
colcon build --packages-select diff_drive_robot --symlink-install
source install/setup.bash
```

## Run

### 1. No hardware at all

Synthesises telemetry internally and pushes it through the real encoder,
CRC, COBS and decoder before publishing:

```bash
ros2 launch diff_drive_robot lora.launch.py fake:=true
```

```bash
ros2 topic echo /lora/telemetry
ros2 topic hz /lora/telemetry        # expect ~2 Hz
ros2 topic echo /lora/link_ok
```

To see the rover move in RViz, add an Odometry display on `/lora/odom` with
**Fixed Frame** set to `odom`.

### 2. No radio, but a real serial port

Exercises the actual serial path over a virtual port pair. `raw,echo=0`
matters: without it the tty rewrites `0x0A` into `0x0D 0x0A` and every frame
containing a newline byte fails CRC.

```bash
sudo apt install socat
socat -d -d pty,raw,echo=0,link=/tmp/ttyROVER pty,raw,echo=0,link=/tmp/ttyBASE
```

```bash
# terminal 2 - stand in for the ESP32
ros2 run diff_drive_robot fake_rover_tx --port /tmp/ttyROVER --rate 2

# terminal 3 - the real base station
ros2 launch diff_drive_robot lora.launch.py port:=/tmp/ttyBASE
```

### 3. Real hardware

Flash `firmware/rover_lora_tx/rover_lora_tx.ino` to the ESP32 (Arduino IDE or
`arduino-cli`, board "ESP32 Dev Module"). `lora_packet.h` must sit next to the
`.ino`, which it already does.

```bash
ros2 launch diff_drive_robot lora.launch.py port:=/dev/ttyUSB0 baudrate:=9600
```

If you get a permission error on the port:

```bash
sudo usermod -aG dialout $USER   # log out and back in
```

Before trusting the radio, confirm the base module works by pointing
`fake_rover_tx` at *it* from a second machine wired to the rover-side module.

## Launch arguments

| Argument | Default | Meaning |
|---|---|---|
| `port` | `/dev/ttyUSB0` | Serial device of the base LoRa module |
| `baudrate` | `9600` | Local UART rate, **not** the over-the-air rate |
| `fake` | `false` | Synthesise telemetry, open no serial port |
| `publish_odometry` | `true` | Also publish `/lora/odom` |
| `use_sim_time` | `false` | Leave false: the radio runs on wall time |

Everything else lives in `config/lora_bridge.yaml`.

## Hardware notes

Written for **UART LoRa modules on both ends** (Ebyte E22/E32, Reyax RYLR998)
in transparent mode: bytes into one module's UART come out of the other's.

- Both radios must share channel, address, air data rate and transmit power.
  Set this once with the vendor tool; mismatched radios receive nothing.
- Ebyte modules use `M0`/`M1` to pick the mode. Both LOW is transparent mode,
  which the firmware assumes. Tie them low or drive them from the ESP32.
- E22 modules can draw over 100 mA while transmitting — more than some
  USB-serial adapters supply. Use a real 3.3 V rail with a bulk capacitor.
  Brownouts here look exactly like range problems.
- `baudrate` is the local wire only. Over-the-air rate is a module setting and
  is usually much slower; that is what actually limits telemetry rate.
- Raising the rate much past 5 Hz will drop packets, and in the EU will breach
  the 1% duty cycle. Slower gives more range.

## Test

To drive the Gazebo robot and watch its pose arrive over the link, see
**[lora_simulation_testing.md](lora_simulation_testing.md)**. It also covers
simulating a lossy link, verifying the EKF and SLAM are unaffected, and
hardware bring-up order.

```bash
colcon test --packages-select diff_drive_robot
colcon test-result --verbose
```

Or directly, with no ROS environment:

```bash
python3 -m pytest test/ -v
```

21 tests cover COBS round-trips, the CRC reference vector, clamping,
resynchronisation after garbage, frames split across reads, and a loopback
over a real pty.

`test_firmware_parity.py` compiles `firmware/test_parity.c` and checks its
output byte-for-byte against `protocol.py`. **If you change the packet format,
change both sides and re-run this** — otherwise the failure mode is a silent
stream of CRC errors over a radio link you cannot attach a debugger to.

## Adding real sensors

`readRoverState()` in the sketch currently integrates a simple differential
drive model, so the link works the moment it is flashed. Replace its body with
encoder and IMU reads; it returns metres, radians, m/s and rad/s, and nothing
downstream changes.

To add a field, update in lockstep: `TELEMETRY_STRUCT` and the pack/unpack
pair in `protocol.py`, `lora_pack_telemetry` in `lora_packet.h`,
`RoverTelemetry.msg`, and the publisher in `lora_receiver_node.py`. Keep the
payload small — every byte costs airtime.
