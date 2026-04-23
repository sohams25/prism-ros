"""
A/B Stress Test: Two separate containers for independent CPU measurement.
Each has its own MediaStreamerNode (10 fps cap to prevent PC overload).
"""

import glob
import os

import psutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


_PACKAGE_MARKER = 'install/prism_image_proc/lib/prism_image_proc'
_FASTDDS_PROFILE = os.path.join(
    get_package_share_directory('prism_image_proc'), 'config', 'fastdds_no_shm.xml')


def _reap_orphans():
    """Kill leftover processes from prior launches of this package.

    `ros2 launch` doesn't always propagate SIGTERM/SIGKILL to child python
    nodes (e.g. latency_tracker.py) on crash or kill -9 of the parent. The
    survivors keep holding Fast-DDS SHM locks, which blocks the next launch's
    SHM init (triggering the RTPS_TRANSPORT_SHM error).
    """
    me = os.getpid()
    for p in psutil.process_iter(['pid', 'cmdline']):
        try:
            if p.info['pid'] == me:
                continue
            cmd = ' '.join(p.info['cmdline'] or [])
            if _PACKAGE_MARKER in cmd:
                p.kill()
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            pass


def _purge_stale_fastdds_shm():
    """Remove /dev/shm/fastrtps_* files not held by any live process.

    Fast-DDS logs an SHM init error and silently falls back to UDP when stale
    locks remain from a prior ROS 2 run. Cleaning them keeps SHM transport
    healthy across restarts. Must run AFTER _reap_orphans so we don't skip
    files held by a zombie we're about to kill.
    """
    live = set()
    for p in psutil.process_iter(['name']):
        try:
            for f in p.open_files():
                if 'fastrtps' in f.path:
                    live.add(f.path)
        except (psutil.NoSuchProcess, psutil.AccessDenied, PermissionError):
            pass
    for path in glob.glob('/dev/shm/fastrtps_*') + glob.glob('/dev/shm/sem.fastrtps_*'):
        if path not in live:
            try:
                os.unlink(path)
            except OSError:
                pass


def generate_launch_description():
    # Disable Fast-DDS SHM transport: UDPv4-only profile avoids the
    # RTPS_TRANSPORT_SHM "Failed init_port" errors from stale /dev/shm locks.
    # Intra-process comms (unique_ptr move) already handle the hot path; this
    # only affects inter-container discovery traffic, which is tiny.
    os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = _FASTDDS_PROFILE

    _reap_orphans()
    _purge_stale_fastdds_shm()

    video_path_arg = DeclareLaunchArgument(
        'video_path', default_value='/tmp/test_video.mp4')

    video_path = LaunchConfiguration('video_path')

    legacy_container = ComposableNodeContainer(
        name='legacy_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='prism_image_proc',
                plugin='prism::MediaStreamerNode',
                name='legacy_source',
                parameters=[{
                    'video_path': video_path,
                    'image_topic': '/legacy/image_raw',
                    'info_topic': '/legacy/camera_info',
                    'max_fps': 10.0,
                }],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='image_proc',
                plugin='image_proc::ResizeNode',
                name='legacy_resize',
                remappings=[
                    ('image/image_raw', '/legacy/image_raw'),
                    ('image/camera_info', '/legacy/camera_info'),
                    ('resize/image_raw', '/legacy/image_processed'),
                    ('resize/camera_info', '/legacy/resize_camera_info'),
                ],
                parameters=[{
                    'use_scale': False,
                    'height': 480,
                    'width': 640,
                    'interpolation': 1,
                }],
            ),
        ],
        output='screen',
        emulate_tty=True,
    )

    accel_container = ComposableNodeContainer(
        name='accel_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='prism_image_proc',
                plugin='prism::MediaStreamerNode',
                name='accel_source',
                parameters=[{
                    'video_path': video_path,
                    'image_topic': '/accelerated/image_raw',
                    'info_topic': '/accelerated/camera_info',
                    'max_fps': 10.0,
                }],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='prism_image_proc',
                plugin='prism::ResizeNode',
                name='accel_resize',
                parameters=[{
                    'input_topic': '/accelerated/image_raw',
                    'output_topic': '/accelerated/image_processed',
                    'action': 'resize',
                    'source_width': 3840,
                    'source_height': 2160,
                    'use_scale': False,
                    'height': 480,
                    'width': 640,
                }],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
        emulate_tty=True,
    )

    latency = Node(
        package='prism_image_proc',
        executable='latency_tracker.py',
        name='latency_tracker',
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', _FASTDDS_PROFILE),
        video_path_arg,
        legacy_container,
        accel_container,
        latency,
    ])
