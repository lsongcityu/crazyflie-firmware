#!/usr/bin/env python3
"""ROS 2 node that publishes the Crazyflie 1 kHz IMU UART stream.

Publishes sensor_msgs/Imu with acceleration in m/s^2 and angular rate in
rad/s. The header stamp is the time of the IMU data ready interrupt in the
time base the firmware announces to the lidar with PPS and GPRMC, so it can be
compared directly with the point timestamps of a synchronized Livox lidar. It
is not the system time of this computer.

    python3 imu_uart_ros2_node.py --ros-args -p port:=/dev/ttyUSB0 -p topic:=/cf/imu
"""

import math

import rclpy
import serial
from rclpy.node import Node
from sensor_msgs.msg import Imu

from imu_uart_protocol import FrameParser

STANDARD_GRAVITY = 9.80665
DEG_TO_RAD = math.pi / 180.0
REPORT_PERIOD_S = 5.0


class ImuUartNode(Node):
    def __init__(self) -> None:
        super().__init__("imu_uart")
        port = self.declare_parameter("port", "/dev/ttyUSB0").value
        baud = self.declare_parameter("baud", 1000000).value
        topic = self.declare_parameter("topic", "/cf/imu").value
        self._frame_id = self.declare_parameter("frame_id", "cf_imu").value

        self._serial = serial.Serial(port, baud, timeout=0.1)
        self._parser = FrameParser()
        self._publisher = self.create_publisher(Imu, topic, 2000)
        self._reported = (0, 0)
        self.create_timer(REPORT_PERIOD_S, self._report)
        self.get_logger().info(f"Reading IMU frames from {port} at {baud} baud, publishing on {topic}")

    def read_and_publish(self) -> None:
        # Return as soon as any data is available to keep the latency low
        data = self._serial.read(self._serial.in_waiting or 1)
        for frame in self._parser.feed(data):
            message = Imu()
            message.header.frame_id = self._frame_id
            message.header.stamp.sec = frame.time_us // 1000000
            message.header.stamp.nanosec = (frame.time_us % 1000000) * 1000
            message.linear_acceleration.x = frame.acc_g[0] * STANDARD_GRAVITY
            message.linear_acceleration.y = frame.acc_g[1] * STANDARD_GRAVITY
            message.linear_acceleration.z = frame.acc_g[2] * STANDARD_GRAVITY
            message.angular_velocity.x = frame.gyro_dps[0] * DEG_TO_RAD
            message.angular_velocity.y = frame.gyro_dps[1] * DEG_TO_RAD
            message.angular_velocity.z = frame.gyro_dps[2] * DEG_TO_RAD
            # No orientation estimate is provided
            message.orientation_covariance[0] = -1.0
            self._publisher.publish(message)

    def _report(self) -> None:
        counters = (self._parser.lost_frames, self._parser.crc_errors)
        lost, crc_errors = (new - old for new, old in zip(counters, self._reported))
        self._reported = counters
        if lost or crc_errors:
            self.get_logger().warn(f"Last {REPORT_PERIOD_S:.0f} s: {lost} frames lost, {crc_errors} CRC errors")


def main() -> None:
    rclpy.init()
    node = ImuUartNode()
    try:
        while rclpy.ok():
            node.read_and_publish()
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
