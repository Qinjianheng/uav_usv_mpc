#!/usr/bin/env python3
"""Local isolated-SITL ground-station heartbeat; never sends flight or parameter commands."""
import signal
import time

from pymavlink import mavutil


def main():
    running = True

    def stop(*_):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    link = mavutil.mavlink_connection('udpin:127.0.0.1:14550', source_system=255)
    last = 0.
    while running:
        message = link.recv_match(blocking=True, timeout=.1)
        if message and message.get_type() == 'HEARTBEAT' and message.get_srcSystem() == 1:
            if time.monotonic()-last >= 1.:
                link.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                        mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                last = time.monotonic()
                print('LOCAL_GCS_HEARTBEAT_SENT', flush=True)
    link.close()


if __name__ == '__main__':
    main()
