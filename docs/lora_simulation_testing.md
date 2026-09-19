# Testing the LoRa bridge in simulation

How to exercise the LoRa telemetry bridge against the Gazebo simulation, with
no ESP32 and no radio, plus what to do once the hardware arrives.

Protocol details and wiring live in [lora_bridge.md](lora_bridge.md). This
file is the test procedure.

## What simulation can and cannot tell you

Worth being clear about up front, because it decides how much the green
results below are actually worth.

| Can be tested in simulation | Needs real hardware |
|---|---|
| Packet format, CRC, COBS framing | RF range and terrain effects |
| Decoder resync after corruption | Antenna and ground-plane behaviour |
| ROS topic wiring, QoS, timestamps | Transmit-power draw and brownouts |
| Watchdog and lost-packet counting | Real duty-cycle and airtime limits |
| Non-interference with EKF/SLAM | Interference from other 433/868/915 traffic |
| Throughput budget arithmetic | Whether your modules are configured alike |

Simulation proves the **software** is right. It says nothing about whether the
radio link works. Those are separate bring-up problems, and it is much easier
to debug them one at a time — which is the point of testing this way first.

## Prerequisites

```bash
sudo apt install socat python3-serial
cd ~/ros2_ws && colcon build --packages-select diff_drive_robot --symlink-install
source install/setup.bash
```

`socat` provides a virtual serial pair. It stands in for the two LoRa modules
and the radio hop between them: bytes written to one end appear at the other,
which is exactly what a pair of modules in transparent mode does.

---

## Test 1 — bridge alone (30 seconds)

Fastest sanity check. No simulator, no serial port. Telemetry is synthesised
internally but still pushed through the real pack, CRC, COBS and decode path.

```bash
ros2 launch diff_drive_robot lora.launch.py fake:=true
```

```bash
ros2 topic echo /lora/telemetry
ros2 topic hz /lora/telemetry      # expect ~2 Hz
```

Expect `seq` incrementing, `x`/`y` changing, `crc_errors: 0`, `packets_lost: 0`.

**This motion is invented.** It is a built-in S-curve unrelated to anything in
Gazebo. Test 1 proves the decoder and publisher work, nothing more. Test 3 is
the one that connects to the actual robot.

---

## Test 2 — real serial path (2 minutes)

Same synthetic motion, but the bytes now cross an actual OS serial device, so
framing, buffering and chunked reads are exercised for real.

```bash
# terminal 1
socat -d -d pty,raw,echo=0,link=/tmp/ttyROVER pty,raw,echo=0,link=/tmp/ttyBASE
```

`raw,echo=0` is not optional. A tty in its default canonical mode rewrites
`0x0A` into `0x0D 0x0A` and echoes input back at you. Leave it out and every
frame containing a newline byte fails CRC, which looks exactly like radio
interference.

```bash
# terminal 2 - stand-in for the ESP32
ros2 run diff_drive_robot fake_rover_tx --port /tmp/ttyROVER --rate 2

# terminal 3 - the real base station
ros2 launch diff_drive_robot lora.launch.py port:=/tmp/ttyBASE
```

```bash
ros2 topic echo /lora/telemetry
```

If `crc_errors` climbs here but Test 1 was clean, the problem is the serial
layer: wrong pty, missing `raw`, or mismatched baud.

---

## Test 3 — the full Gazebo loop

The real one. Drive the simulated robot with the keyboard and watch **its**
pose arrive over the LoRa path:

```
Gazebo -> /odometry/filtered -> sim_rover_tx -> pty --(socat)--> pty
                                                                  |
                                     /lora/telemetry <- lora_receiver
```

`sim_rover_tx` is the test-only stand-in for the ESP32: it subscribes to the
simulator's odometry and emits exactly the frames the firmware emits. On real
hardware the ESP32 does this and `sim_rover_tx` does not run.

**Terminal 1 — simulation** (Gazebo, robot state publisher, gz bridge, EKF):

```bash
ros2 launch diff_drive_robot robot.launch.py
```

**Terminal 2 — the virtual radio pair:**

```bash
socat -d -d pty,raw,echo=0,link=/tmp/ttyROVER pty,raw,echo=0,link=/tmp/ttyBASE
```

**Terminal 3 — rover side:**

```bash
ros2 run diff_drive_robot sim_rover_tx --ros-args \
  -p port:=/tmp/ttyROVER \
  -p odom_topic:=/odometry/filtered \
  -p rate:=2.0 \
  -p use_sim_time:=true
```

Use `odom_topic:=/odom` for raw wheel odometry instead of the EKF's fused
estimate. The EKF output is the better test — it is what a real rover would
transmit.

**Terminal 4 — base station:**

```bash
ros2 launch diff_drive_robot lora.launch.py \
  port:=/tmp/ttyBASE use_sim_time:=true
```

> **Set `use_sim_time:=true` on both.** It defaults to false because real radio
> hardware runs on wall time, but in this test the data originates in the
> simulator. Mismatch them and RViz silently discards `/lora/odom` with
> "message removed because it is too old" — the topic looks alive in
> `ros2 topic echo` but nothing ever renders.

**Terminal 5 — drive it:**

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### What success looks like

Drive forward with `i`, turn with `j`/`l`, and compare:

```bash
ros2 topic echo /odometry/filtered --once   # ground truth in the sim
ros2 topic echo /lora/telemetry --once      # what came over the "radio"
```

`x` and `y` should agree to within about a millimetre, `yaw` to within
0.01 degrees. Those are the fixed-point quantisation limits from
[lora_bridge.md](lora_bridge.md), not error.

To see it, start RViz — note that `robot.launch.py` has its RViz node
commented out on this branch, so run one of:

```bash
ros2 launch diff_drive_robot mapping.launch.py     # RViz + SLAM Toolbox
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix diff_drive_robot)/share/diff_drive_robot/rviz/mapping.rviz
```

Add an **Odometry** display on `/lora/odom` with **Fixed Frame** set to
`odom`. It tracks the robot in coarse 2 Hz steps while the simulator's own
odometry moves smoothly. That stepping is the honest picture of what a LoRa
link gives you, and is the main reason to look at this before committing to a
telemetry rate.

---

## Test 4 — prove nothing else broke

The bridge is meant to be invisible to the rest of the stack. Verify rather
than assume, with Test 3 still running.

```bash
# Nothing subscribes to the robot's control or sensor topics from /lora
ros2 topic info /cmd_vel --verbose
ros2 topic info /scan --verbose

# The bridge owns nothing outside its namespace
ros2 node info /lora_receiver

# No TF frames were injected
ros2 run tf2_tools view_frames
```

`/lora_receiver` should show **zero subscriptions** and only `/lora/*`
publishers. The TF tree should contain `map`, `odom`, `base_link` and the
sensor frames — and no `lora_rover`.

Then run SLAM alongside it and confirm mapping is unaffected:

```bash
ros2 launch diff_drive_robot mapping.launch.py
```

Drive around. The map should build exactly as it does with the bridge stopped.

---

## Test 5 — simulate a bad link

A perfect link never exercises the watchdog or the lost-packet counter, which
are precisely the parts you depend on when the rover is far away. Force the
issue:

```bash
ros2 run diff_drive_robot sim_rover_tx --ros-args \
  -p port:=/tmp/ttyROVER -p odom_topic:=/odometry/filtered \
  -p drop_probability:=0.3 -p use_sim_time:=true
```

```bash
ros2 topic echo /lora/telemetry    # packets_lost should climb steadily
ros2 topic echo /lora/link_ok      # heartbeat at 1 Hz
```

The sequence number still advances on a dropped packet, which is what makes
the gap visible to the receiver — the same way a frame lost to interference
would be.

To test link loss outright, stop `sim_rover_tx` (Ctrl-C). `/lora/link_ok`
should go `false` within `link_timeout` (default 5 s) and log `link down`.
Restart it and it returns to `true`.

---

## Throughput reality check

Simulation makes it tempting to raise the rate, because a pty has no bandwidth
limit. A radio does.

```bash
ros2 topic hz /lora/telemetry
ros2 topic bw /lora/telemetry
```

`ros2 topic bw` reports the **ROS message** size, not the wire size. On the air
each packet is 29 bytes. Rough airtime for that at 125 kHz bandwidth:

| Spreading factor | Airtime per packet | Max sustainable rate | Range |
|---|---|---|---|
| SF7 | ~60 ms | comfortably 2–5 Hz | shortest |
| SF9 | ~190 ms | ~2 Hz | medium |
| SF12 | ~1.3 s | below 1 Hz | longest |

So 2 Hz is fine at SF7–SF9 and **impossible at SF12** — the packet takes longer
to send than the interval between packets. Where a 1% duty cycle applies
(most EU 868 MHz sub-bands), SF12 at 1.3 s airtime allows roughly one packet
every two minutes. Pick the rate to match the spreading factor you need for
your range, not the other way round.

---

## Troubleshooting

| Symptom | Cause |
|---|---|
| No `/lora/telemetry` at all | socat not running, or `port` points at the wrong pty |
| `could not open /tmp/ttyROVER` | Start socat first; it creates the links |
| `crc_errors` climbing | `raw,echo=0` missing from socat, or baud mismatch |
| `no odometry received yet` | Wrong `odom_topic`; check `ros2 topic list` |
| Topic alive but nothing in RViz | `use_sim_time` mismatch — set it true on both nodes |
| `link_ok` flapping | `rate` too low relative to `link_timeout` |
| Pose steps rather than glides | Expected: 2 Hz updates plus fixed-point quantisation |
| `Permission denied: /dev/ttyUSB0` | `sudo usermod -aG dialout $USER`, then re-login |

---

## Further steps

### 1. Real sensors on the rover

`readRoverState()` in `firmware/rover_lora_tx/rover_lora_tx.ino` currently
integrates a kinematic model so the link works the moment it is flashed.
Replace its body with encoder and IMU reads — convert ticks to wheel
velocities using wheel radius and ticks per revolution, combine into body
velocity using wheel separation, integrate for pose. It returns metres,
radians, m/s and rad/s, and nothing downstream changes.

### 2. Hardware bring-up, one layer at a time

Do not flash the ESP32 and power the radios in the same step. When it fails
you will not know which half is wrong.

1. **Configure both radios identically** — channel, address, air data rate,
   transmit power — with the vendor tool. Mismatched modules receive nothing
   and give no error.
2. **Prove the base module alone.** Wire a second computer to the rover-side
   module and point `fake_rover_tx` at it. If frames reach `/lora/telemetry`,
   the radio path and the base station both work.
3. **Then flash the ESP32** and substitute it for that computer. Anything that
   breaks now is firmware or the ESP32's UART wiring.
4. **Range test last**, starting a few metres apart and walking out, watching
   `packets_lost` and `crc_errors` rather than trusting a signal bar.

Brownouts look exactly like range problems. An E22 can pull over 100 mA while
transmitting; if `crc_errors` climbs the instant you raise transmit power,
suspect the supply before the antenna.

### 3. Adding telemetry fields

Every byte costs airtime, so add deliberately. Five places must change in
lockstep:

1. `TELEMETRY_STRUCT` and the pack/unpack pair in `lora_bridge/protocol.py`
2. `lora_pack_telemetry` in `firmware/rover_lora_tx/lora_packet.h`
3. `msg/RoverTelemetry.msg`
4. The publisher in `lora_bridge/lora_receiver_node.py`
5. The `SAMPLES` list in `test/test_firmware_parity.py`

Then run `python3 -m pytest test/ -v`. The parity test compiles the firmware
and diffs its bytes against the Python; if you changed one side only, it fails
there instead of over a radio link you cannot attach a debugger to.

Battery voltage and a status/fault byte are the usual next additions — about
6 bytes, and they answer "is the rover in trouble" which pose alone does not.

### 4. If you later want a downlink

Commanding the rover over LoRa was deliberately left out: you asked for
telemetry only, and half-duplex radios plus `/cmd_vel` streaming is a much
larger problem than it looks. If you do want it:

- The `msg_id` byte already in the header means a second message type needs no
  change to the framing.
- The rover needs a **failsafe watchdog** — stop the motors if no command
  arrives within a timeout. Without it, a rover that loses the link keeps
  executing its last command indefinitely.
- Send discrete commands (stop, go to waypoint, set rate) rather than streaming
  velocity. At SF9, a velocity stream cannot arrive fast enough to steer by,
  and it will saturate the same channel your telemetry needs.
- Transmitting on the rover means it cannot receive, so the two directions must
  be scheduled rather than both talking freely.
