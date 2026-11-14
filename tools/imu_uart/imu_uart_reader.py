#!/usr/bin/env python3
"""Read the Crazyflie 1 kHz IMU UART stream and report link quality.

Prints one line per second with the frame rate, the number of lost frames and
CRC errors, the spread of the sample interval and the largest acceleration and
angular rate seen. Optionally logs every frame to a CSV file.

    pip install pyserial
    python3 imu_uart_reader.py /dev/ttyUSB0
    python3 imu_uart_reader.py /dev/ttyUSB0 --csv imu.csv
"""

import argparse
import csv
import math
import sys
import time

from imu_uart_protocol import FrameParser

# Full scale of the BMI088 as configured in the firmware
ACC_FULL_SCALE_G = 24.0
GYRO_FULL_SCALE_DPS = 2000.0
NEAR_FULL_SCALE = 0.98


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("port", help="serial port, for example /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=1000000, help="baud rate (default: %(default)s)")
    parser.add_argument("--csv", help="write every frame to this CSV file")
    args = parser.parse_args()

    try:
        import serial
    except ImportError:
        print("pyserial is missing, install it with: pip install pyserial", file=sys.stderr)
        return 1

    port = serial.Serial(args.port, args.baud, timeout=0.05)
    frame_parser = FrameParser()

    csv_file = open(args.csv, "w", newline="") if args.csv else None
    writer = csv.writer(csv_file) if csv_file else None
    if writer:
        writer.writerow(["seq", "time_us", "acc_x_g", "acc_y_g", "acc_z_g",
                         "gyro_x_dps", "gyro_y_dps", "gyro_z_dps", "flags"])

    print("    rate    lost  crc_err   dt_min   dt_max  max|acc|  max|gyro|  near_full_scale")
    print("    [Hz]                      [us]     [us]       [G]    [deg/s]        [samples]")

    last_report = time.monotonic()
    last_counters = (0, 0, 0)
    last_time_us = None
    dt_min = dt_max = None
    max_acc = max_gyro = 0.0
    near_full_scale = 0

    try:
        while True:
            for frame in frame_parser.feed(port.read(4096)):
                if last_time_us is not None:
                    dt = frame.time_us - last_time_us
                    dt_min = dt if dt_min is None else min(dt_min, dt)
                    dt_max = dt if dt_max is None else max(dt_max, dt)
                last_time_us = frame.time_us

                max_acc = max(max_acc, math.sqrt(sum(a * a for a in frame.acc_g)))
                max_gyro = max(max_gyro, math.sqrt(sum(g * g for g in frame.gyro_dps)))
                if (max(abs(a) for a in frame.acc_g) > NEAR_FULL_SCALE * ACC_FULL_SCALE_G
                        or max(abs(g) for g in frame.gyro_dps) > NEAR_FULL_SCALE * GYRO_FULL_SCALE_DPS):
                    near_full_scale += 1

                if writer:
                    writer.writerow([frame.seq, frame.time_us, *frame.acc_g, *frame.gyro_dps, frame.flags])

            now = time.monotonic()
            if now - last_report >= 1.0:
                counters = (frame_parser.frames, frame_parser.lost_frames, frame_parser.crc_errors)
                frames, lost, crc_errors = (new - old for new, old in zip(counters, last_counters))
                rate = frames / (now - last_report)
                dt_text = f"{dt_min:8d} {dt_max:8d}" if dt_min is not None else f"{'-':>8} {'-':>8}"
                print(f"{rate:8.1f} {lost:7d} {crc_errors:8d} {dt_text} {max_acc:9.2f} {max_gyro:10.1f} "
                      f"{near_full_scale:16d}")

                last_report = now
                last_counters = counters
                dt_min = dt_max = None
                max_acc = max_gyro = 0.0
                near_full_scale = 0
    except KeyboardInterrupt:
        pass
    finally:
        if csv_file:
            csv_file.close()
        port.close()

    print(f"\ntotal: {frame_parser.frames} frames, {frame_parser.lost_frames} lost, "
          f"{frame_parser.crc_errors} CRC errors, {frame_parser.skipped_bytes} bytes skipped")
    return 0


if __name__ == "__main__":
    sys.exit(main())
