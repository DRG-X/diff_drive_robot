#!/usr/bin/env python3
"""Stand-in for the ESP32: writes real telemetry frames to a serial port.

No ROS dependency, so it runs anywhere python3 and pyserial do -- including a
laptop wired to the rover-side LoRa module. Two ways to use it:

  * over a virtual serial pair, to test the base station with no hardware::

        socat -d -d pty,raw,echo=0,link=/tmp/ttyROVER pty,raw,echo=0,link=/tmp/ttyBASE
        python3 fake_rover_tx.py --port /tmp/ttyROVER

  * over a real LoRa module, to prove the radio path before the ESP32 exists.

The frames it emits are byte-for-byte what firmware/rover_lora_tx produces.
"""

import argparse
import sys
import time

try:
    from lora_bridge.protocol import MSG_ID_TELEMETRY, encode_frame, pack_telemetry
    from lora_bridge.rover_sim import SimulatedRover
except ImportError:  # running straight from a source checkout
    import os
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from lora_bridge.protocol import MSG_ID_TELEMETRY, encode_frame, pack_telemetry
    from lora_bridge.rover_sim import SimulatedRover


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--port', required=True,
                        help='serial device to write frames to, e.g. /tmp/ttyROVER')
    parser.add_argument('--baud', type=int, default=9600,
                        help='baud rate of the local UART link (default: 9600)')
    parser.add_argument('--rate', type=float, default=2.0,
                        help='telemetry packets per second (default: 2.0)')
    parser.add_argument('--speed', type=float, default=0.25,
                        help='simulated forward speed in m/s (default: 0.25)')
    parser.add_argument('--count', type=int, default=0,
                        help='stop after N packets (default: 0, run forever)')
    args = parser.parse_args()

    try:
        import serial
    except ImportError:
        sys.exit('pyserial is not installed. Run: sudo apt install python3-serial')

    period = 1.0 / args.rate
    rover = SimulatedRover(linear_speed=args.speed)

    with serial.Serial(args.port, args.baud, timeout=1) as link:
        print('writing %.1f Hz telemetry to %s at %d baud (Ctrl-C to stop)'
              % (args.rate, args.port, args.baud))
        sent = 0
        while args.count == 0 or sent < args.count:
            state = rover.step(period)
            link.write(encode_frame(MSG_ID_TELEMETRY, pack_telemetry(**state)))
            link.flush()
            sent += 1
            print('\rseq %-8d x=%7.2f y=%7.2f yaw=%7.3f' % (
                state['seq'], state['x'], state['y'], state['yaw']), end='')
            sys.stdout.flush()
            time.sleep(period)
        print()


if __name__ == '__main__':
    main()
