# Copyright 2026 Prism contributors. Apache-2.0.
"""launch_testing integration test for prism::ImageProcNode direct mode.

Launches the standalone prism_image_proc node, publishes synthetic bgr8 frames
plus a CameraInfo on the input topics, and asserts that processed frames come
out at the configured 640x480 bgr8 size and that the republished CameraInfo is
transformed to match. This exercises the real runtime data path (subscription
-> direct-mode cv::resize -> publish, plus CameraInfo plumbing) that the
PipelineFactory unit tests cannot reach.

On a host with no working GPU GStreamer element (CI, most desktops) the node
selects direct mode automatically, which is exactly the path under test.
"""
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

IN_W, IN_H = 1280, 720
OUT_W, OUT_H = 640, 480  # node defaults


@pytest.mark.launch_test
def generate_test_description():
    node = launch_ros.actions.Node(
        package='prism_image_proc',
        executable='prism_image_proc',
        name='image_proc_node',
        output='screen',
    )
    return (
        launch.LaunchDescription([
            launch.actions.SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
            node,
            launch_testing.actions.ReadyToTest(),
        ]),
        {'node': node},
    )


class TestDirectMode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('prism_integration_probe')

    def tearDown(self):
        self.node.destroy_node()

    def test_direct_mode_resizes_and_transforms(self):
        import numpy as np

        pub_img = self.node.create_publisher(Image, '/camera/image_raw', 10)
        pub_info = self.node.create_publisher(CameraInfo, '/camera/camera_info', 10)

        recv = {'img': 0, 'info': 0, 'dims': None, 'enc': None, 'info_wh': None,
                'frame_id': None}

        def on_img(msg):
            recv['img'] += 1
            recv['dims'] = (msg.width, msg.height)
            recv['enc'] = msg.encoding
            recv['frame_id'] = msg.header.frame_id

        def on_info(msg):
            if msg.width == OUT_W:  # the node's transformed (output) CameraInfo
                recv['info'] += 1
                recv['info_wh'] = (msg.width, msg.height)

        self.node.create_subscription(
            Image, '/camera/image_processed', on_img, qos_profile_sensor_data)
        self.node.create_subscription(
            CameraInfo, '/camera/camera_info', on_info, qos_profile_sensor_data)

        frame = np.zeros((IN_H, IN_W, 3), dtype=np.uint8)
        frame[:, :, 1] = 128

        def make_image(stamp):
            m = Image()
            m.header.stamp = stamp
            m.header.frame_id = 'itest'
            m.height, m.width = IN_H, IN_W
            m.encoding = 'bgr8'
            m.is_bigendian = False
            m.step = IN_W * 3
            m.data = frame.tobytes()
            return m

        def make_info(stamp):
            ci = CameraInfo()
            ci.header.stamp = stamp
            ci.header.frame_id = 'itest'
            ci.width, ci.height = IN_W, IN_H
            ci.distortion_model = 'plumb_bob'
            ci.d = [0.0, 0.0, 0.0, 0.0, 0.0]
            fx = float(IN_W)
            ci.k = [fx, 0.0, IN_W / 2.0, 0.0, fx, IN_H / 2.0, 0.0, 0.0, 1.0]
            ci.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            ci.p = [fx, 0.0, IN_W / 2.0, 0.0, 0.0, fx, IN_H / 2.0, 0.0,
                    0.0, 0.0, 1.0, 0.0]
            return ci

        # Pump frames for up to ~12 s or until we have enough evidence.
        deadline = self.node.get_clock().now().nanoseconds + 12_000_000_000
        next_pub = 0
        while self.node.get_clock().now().nanoseconds < deadline:
            now = self.node.get_clock().now().nanoseconds
            if now >= next_pub:
                stamp = self.node.get_clock().now().to_msg()
                pub_img.publish(make_image(stamp))
                pub_info.publish(make_info(stamp))
                next_pub = now + 100_000_000  # 10 Hz
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if recv['img'] >= 5 and recv['info'] >= 1:
                break

        self.assertGreaterEqual(
            recv['img'], 3, 'no processed frames received from the node')
        self.assertEqual(recv['dims'], (OUT_W, OUT_H), 'output not resized to 640x480')
        self.assertEqual(recv['enc'], 'bgr8', 'output encoding is not bgr8')
        self.assertEqual(recv['frame_id'], 'itest',
                         'input frame_id not preserved on the output image')
        self.assertGreaterEqual(
            recv['info'], 1, 'no transformed CameraInfo received')
        self.assertEqual(recv['info_wh'], (OUT_W, OUT_H),
                         'CameraInfo not transformed to output size')


@launch_testing.post_shutdown_test()
class TestProcessExit(unittest.TestCase):

    def test_clean_exit(self, proc_info):
        # The node is SIGINT-terminated by launch_testing on shutdown; accept
        # the conventional clean-exit codes for a signal-terminated ROS node.
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=[0, -2, 130, 15, -15])
