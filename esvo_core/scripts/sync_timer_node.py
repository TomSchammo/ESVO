#!/usr/bin/env python3
"""
Sync timer node for esvo_time_surface.

Publishes the current time to /sync at a configurable rate.
When use_sim_time is enabled, this publishes sim time from /clock,
which is required for rosbag playback synchronization.
"""

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time


class SyncTimerNode(Node):
    def __init__(self):
        super().__init__('sync_timer')

        # Declare parameters
        self.declare_parameter('rate_hz', 50.0)

        rate_hz = self.get_parameter('rate_hz').get_parameter_value().double_value

        self.publisher_ = self.create_publisher(Time, 'sync', 1)
        self.timer = self.create_timer(1.0 / rate_hz, self.timer_callback)

        self.get_logger().info(
            f'Sync timer started at {rate_hz} Hz '
            f'(use_sim_time={self.get_parameter("use_sim_time").get_parameter_value().bool_value})'
        )

    def timer_callback(self):
        # When use_sim_time=true, get_clock().now() returns sim time from /clock
        msg = self.get_clock().now().to_msg()
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SyncTimerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
