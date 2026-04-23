#!/usr/bin/env python3
"""
Benchmark partner only, not a production node.

Minimal rclpy node that acts as the fairest available approximation
of a stock "colorconvert" step: subscribe to sensor_msgs/Image, do
cv_bridge toCvCopy + toImageMsg in the target encoding, re-publish.

Used by launch/A_B_comparison.launch.py on the stock (legacy) side
for the `colorconvert` and `chain` operations, where image_proc has
no dedicated colorconvert plugin.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class CvBridgeSubscriber(Node):

    def __init__(self):
        super().__init__('cv_bridge_subscriber')

        self.declare_parameter('input_topic', '/legacy/image_raw')
        self.declare_parameter('output_topic', '/legacy/image_processed')
        self.declare_parameter('target_encoding', 'rgb8')

        self.input_topic_ = self.get_parameter('input_topic').value
        self.output_topic_ = self.get_parameter('output_topic').value
        self.target_encoding_ = self.get_parameter('target_encoding').value

        self.bridge_ = CvBridge()
        self.pub_ = self.create_publisher(Image, self.output_topic_, 10)
        self.sub_ = self.create_subscription(
            Image, self.input_topic_, self.on_image, 10)

        self.get_logger().info(
            f'cv_bridge_subscriber: {self.input_topic_} -> '
            f'{self.output_topic_} (encoding={self.target_encoding_})')

    def on_image(self, msg):
        cv_img = self.bridge_.imgmsg_to_cv2(msg, desired_encoding=self.target_encoding_)
        out = self.bridge_.cv2_to_imgmsg(cv_img, encoding=self.target_encoding_)
        out.header = msg.header  # preserve timestamp + frame_id for latency math
        self.pub_.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CvBridgeSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
