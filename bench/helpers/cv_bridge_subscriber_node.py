#!/usr/bin/python3
"""
Benchmark partner only, not a production node.

Minimal rclpy node that acts as the fairest available approximation
of a stock "colorconvert" step: subscribe to sensor_msgs/Image,
reshape the payload with NumPy, flip channels / reduce to mono to
match the requested encoding, and re-publish.

The original intent was to call cv_bridge::toCvCopy here. On this
machine cv_bridge's native Boost extension was built against NumPy
1.x and segfaults under NumPy 2.x, so the helper dropped the
cv_bridge import and does the pixel-level conversion with NumPy
directly. The operations involved (bgr8 ↔ rgb8 channel reorder,
mono8 luma reduction) are simple enough that a NumPy implementation
is representative of what a ROS 2 user would reach for when they
need a standalone colorconvert step — many projects skip cv_bridge
for these trivial cases. See bench/METHODOLOGY.md for the caveat.

Used by launch/A_B_comparison.launch.py on the stock (legacy) side
for the `colorconvert` and `chain` operations, where image_proc has
no dedicated colorconvert plugin.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


_STEP_BYTES_PER_PX = {'rgb8': 3, 'bgr8': 3, 'mono8': 1}


def _convert(buf, src_enc, tgt_enc):
    if tgt_enc == src_enc:
        return buf.copy()

    if {src_enc, tgt_enc} == {'rgb8', 'bgr8'}:
        return buf[:, :, ::-1].copy()

    if tgt_enc == 'mono8':
        if src_enc == 'bgr8':
            # Rec.601 luma, B/G/R order on input.
            return (buf[:, :, 0] * 0.114
                    + buf[:, :, 1] * 0.587
                    + buf[:, :, 2] * 0.299).astype(np.uint8)
        if src_enc == 'rgb8':
            return (buf[:, :, 0] * 0.299
                    + buf[:, :, 1] * 0.587
                    + buf[:, :, 2] * 0.114).astype(np.uint8)

    # Fallback: passthrough, unchanged data.
    return buf.copy()


class ColorConvertSubscriber(Node):

    def __init__(self):
        super().__init__('cv_bridge_subscriber')

        self.declare_parameter('input_topic', '/legacy/image_raw')
        self.declare_parameter('output_topic', '/legacy/image_processed')
        self.declare_parameter('target_encoding', 'rgb8')

        self.input_topic_ = self.get_parameter('input_topic').value
        self.output_topic_ = self.get_parameter('output_topic').value
        self.target_encoding_ = self.get_parameter('target_encoding').value

        self.pub_ = self.create_publisher(Image, self.output_topic_, 10)
        self.sub_ = self.create_subscription(
            Image, self.input_topic_, self.on_image, 10)

        self.get_logger().info(
            f'colorconvert subscriber: {self.input_topic_} -> '
            f'{self.output_topic_} (encoding={self.target_encoding_}, numpy backend)')

    def on_image(self, msg):
        src_enc = msg.encoding or 'bgr8'
        h, w = msg.height, msg.width
        buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, -1)

        out_arr = _convert(buf, src_enc, self.target_encoding_)

        out = Image()
        out.header = msg.header
        out.height = h
        out.width = w
        out.encoding = self.target_encoding_
        out.is_bigendian = msg.is_bigendian
        out.step = w * _STEP_BYTES_PER_PX.get(self.target_encoding_, out_arr.shape[-1])
        out.data = out_arr.tobytes()
        self.pub_.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ColorConvertSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
