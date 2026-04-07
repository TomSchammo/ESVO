#!/usr/bin/env python3
"""
Camera Info Publisher for ESVO

Reads calibration YAML files and publishes CameraInfo messages.
Easily configurable for different datasets by changing the calib_dir parameter.

Usage:
  ros2 run esvo_time_surface camera_info_publisher.py --ros-args \
    -p calib_dir:=/path/to/calib/rpg \
    -p frame_id:=dvs \
    -p publish_rate:=100.0
"""

import os
import yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo


class CameraInfoPublisher(Node):

    def __init__(self):
        super().__init__('camera_info_publisher')

        # Declare parameters
        self.declare_parameter('calib_dir', '')
        self.declare_parameter('frame_id', 'dvs')
        self.declare_parameter('publish_rate', 100.0)
        self.declare_parameter('left_topic', '/evk/left/camera_info')
        self.declare_parameter('right_topic', '/evk/right/camera_info')

        # Get parameters
        calib_dir = self.get_parameter(
            'calib_dir').get_parameter_value().string_value
        frame_id = self.get_parameter(
            'frame_id').get_parameter_value().string_value
        publish_rate = self.get_parameter(
            'publish_rate').get_parameter_value().double_value
        left_topic = self.get_parameter(
            'left_topic').get_parameter_value().string_value
        right_topic = self.get_parameter(
            'right_topic').get_parameter_value().string_value

        if not calib_dir:
            self.get_logger().error('calib_dir parameter is required!')
            raise ValueError('calib_dir parameter is required')

        # Load calibration files
        left_yaml = os.path.join(calib_dir, 'left.yaml')
        right_yaml = os.path.join(calib_dir, 'right.yaml')

        self.get_logger().info(f'Loading calibration from: {calib_dir}')

        self.left_msg = self._load_camera_info(left_yaml, frame_id)
        self.right_msg = self._load_camera_info(right_yaml, frame_id)

        if self.left_msg is None or self.right_msg is None:
            raise RuntimeError('Failed to load calibration files')

        # Create publishers
        self.left_pub = self.create_publisher(CameraInfo, left_topic, 10)
        self.right_pub = self.create_publisher(CameraInfo, right_topic, 10)

        # Create timer
        period = 1.0 / publish_rate
        self.timer = self.create_timer(period, self._publish_callback)

        self.get_logger().info(
            f'Publishing camera info at {publish_rate} Hz on {left_topic} and {right_topic}'
        )

    def _load_camera_info(self, yaml_path: str, frame_id: str) -> CameraInfo:
        """Load CameraInfo from a YAML calibration file."""
        if not os.path.exists(yaml_path):
            self.get_logger().error(f'Calibration file not found: {yaml_path}')
            return None

        try:
            with open(yaml_path, 'r') as f:
                calib = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f'Failed to parse {yaml_path}: {e}')
            return None

        msg = CameraInfo()
        msg.header.frame_id = frame_id

        # Image dimensions
        msg.width = calib.get('image_width', 0)
        msg.height = calib.get('image_height', 0)

        # Distortion model
        msg.distortion_model = calib.get('distortion_model', 'plumb_bob')

        # Distortion coefficients (D)
        d_data = calib.get('distortion_coefficients', {}).get('data', [])
        # Ensure 5 elements for plumb_bob model
        while len(d_data) < 5:
            d_data.append(0.0)
        msg.d = [float(x) for x in d_data[:5]]

        # Camera matrix (K) - 3x3 -> 9 elements
        k_data = calib.get('camera_matrix', {}).get('data', [0] * 9)
        msg.k = [float(x) for x in k_data[:9]]

        # Rectification matrix (R) - 3x3 -> 9 elements
        r_data = calib.get('rectification_matrix',
                           {}).get('data', [1, 0, 0, 0, 1, 0, 0, 0, 1])
        msg.r = [float(x) for x in r_data[:9]]

        # Projection matrix (P) - 3x4 -> 12 elements
        p_data = calib.get('projection_matrix', {}).get('data', [0] * 12)
        msg.p = [float(x) for x in p_data[:12]]

        self.get_logger().info(
            f'Loaded {yaml_path}: {msg.width}x{msg.height}, model={msg.distortion_model}'
        )

        return msg

    def _publish_callback(self):
        """Publish camera info messages with current timestamp."""
        now = self.get_clock().now().to_msg()

        self.left_msg.header.stamp = now
        self.right_msg.header.stamp = now

        self.left_pub.publish(self.left_msg)
        self.right_pub.publish(self.right_msg)


def main(args=None):
    rclpy.init(args=args)

    try:
        node = CameraInfoPublisher()
        rclpy.spin(node)
    except (ValueError, RuntimeError) as e:
        print(f'Error: {e}')
        return 1
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

    return 0


if __name__ == '__main__':
    exit(main())
