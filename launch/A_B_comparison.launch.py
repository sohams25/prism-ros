"""
A/B Stress Test — two separate containers for independent CPU measurement.

Defaults to operation:=resize, which preserves the original behaviour
byte-for-byte: one image_proc::ResizeNode and one prism::ResizeNode,
fed by a shared MediaStreamerNode on each side at 10 fps. The
`operation` launch argument (introduced for v0.1.0 benchmarking)
extends the launch to cover crop, colorconvert, and the full
crop,resize,colorconvert chain — each mapped to the closest
available stock-ROS equivalent on the legacy side. See
bench/operations.yaml and bench/METHODOLOGY.md for the full mapping.
"""

import glob
import os

import psutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


_PACKAGE_MARKER = 'install/prism_image_proc/lib/prism_image_proc'
_FASTDDS_PROFILE = os.path.join(
    get_package_share_directory('prism_image_proc'), 'config', 'fastdds_no_shm.xml')

# Intermediate topic names used only by the `chain` operation on the
# stock side, where three separate nodes are DDS-piped together.
_STOCK_CHAIN_CROP_OUT = '/legacy/_chain_cropped'
_STOCK_CHAIN_RESIZE_OUT = '/legacy/_chain_resized'


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
    """Remove /dev/shm/fastrtps_* files not held by any live process."""
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


# --------------------------------------------------------------------------
# Source node (shared by every operation, identical on both sides)
# --------------------------------------------------------------------------
def _source_node(name, image_topic, info_topic, video_path, intra_process,
                 extra_params=None):
    params = {
        'video_path': video_path,
        'image_topic': image_topic,
        'info_topic': info_topic,
        'max_fps': 10.0,
    }
    if extra_params:
        params.update(extra_params)
    return ComposableNode(
        package='prism_image_proc',
        plugin='prism::MediaStreamerNode',
        name=name,
        parameters=[params],
        extra_arguments=[{'use_intra_process_comms': intra_process}],
    )


# --------------------------------------------------------------------------
# Stock-side graphs (one per operation)
# --------------------------------------------------------------------------
def _stock_resize_nodes(video_path):
    return [
        _source_node('legacy_source',
                     '/legacy/image_raw', '/legacy/camera_info',
                     video_path, True),
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
    ], []  # (composable_nodes, standalone_nodes)


def _stock_crop_nodes(video_path):
    return [
        _source_node('legacy_source',
                     '/legacy/image_raw', '/legacy/camera_info',
                     video_path, True),
        ComposableNode(
            package='image_proc',
            plugin='image_proc::CropDecimateNode',
            name='legacy_crop',
            remappings=[
                ('in/image_raw', '/legacy/image_raw'),
                ('in/camera_info', '/legacy/camera_info'),
                ('out/image_raw', '/legacy/image_processed'),
                ('out/camera_info', '/legacy/crop_camera_info'),
            ],
            parameters=[{
                'x_offset': 640,
                'y_offset': 360,
                'width': 2560,
                'height': 1440,
                'decimation_x': 1,
                'decimation_y': 1,
            }],
        ),
    ], []


def _stock_colorconvert_nodes(video_path):
    # Stock side: MediaStreamer only. The cv_bridge Python node runs
    # as a standalone process (see _stock_cvbridge_node below) and
    # closes the /legacy/image_processed loop.
    return [
        _source_node('legacy_source',
                     '/legacy/image_raw', '/legacy/camera_info',
                     video_path, True),
    ], [
        _stock_cvbridge_node(
            input_topic='/legacy/image_raw',
            output_topic='/legacy/image_processed',
            target_encoding='rgb8'),
    ]


_RECTIFY_SOURCE_OVERRIDES = {
    'focal_x': 1900.0,
    'focal_y': 1900.0,
    'distortion_d': [-0.30, 0.10, 0.0, 0.0, -0.02],
}


def _stock_rectify_nodes(video_path):
    return [
        _source_node('legacy_source',
                     '/legacy/image_raw', '/legacy/camera_info',
                     video_path, True,
                     extra_params=_RECTIFY_SOURCE_OVERRIDES),
        ComposableNode(
            package='image_proc',
            plugin='image_proc::RectifyNode',
            name='legacy_rectify',
            remappings=[
                ('image', '/legacy/image_raw'),
                ('camera_info', '/legacy/camera_info'),
                ('image_rect', '/legacy/image_processed'),
            ],
            parameters=[{
                'interpolation': 1,
            }],
        ),
    ], []


def _stock_chain_nodes(video_path):
    # Chain stock side — three separate nodes DDS-piped through two
    # intermediate topics. Intra-process off on everything downstream
    # of the source so each hop actually serialises through DDS.
    return [
        _source_node('legacy_source',
                     '/legacy/image_raw', '/legacy/camera_info',
                     video_path, False),
        ComposableNode(
            package='image_proc',
            plugin='image_proc::CropDecimateNode',
            name='legacy_chain_crop',
            remappings=[
                ('in/image_raw', '/legacy/image_raw'),
                ('in/camera_info', '/legacy/camera_info'),
                ('out/image_raw', _STOCK_CHAIN_CROP_OUT),
                ('out/camera_info', '/legacy/_chain_cropped_info'),
            ],
            parameters=[{
                'x_offset': 640,
                'y_offset': 360,
                'width': 2560,
                'height': 1440,
                'decimation_x': 1,
                'decimation_y': 1,
            }],
            extra_arguments=[{'use_intra_process_comms': False}],
        ),
        ComposableNode(
            package='image_proc',
            plugin='image_proc::ResizeNode',
            name='legacy_chain_resize',
            remappings=[
                ('image/image_raw', _STOCK_CHAIN_CROP_OUT),
                ('image/camera_info', '/legacy/_chain_cropped_info'),
                ('resize/image_raw', _STOCK_CHAIN_RESIZE_OUT),
                ('resize/camera_info', '/legacy/_chain_resized_info'),
            ],
            parameters=[{
                'use_scale': False,
                'height': 480,
                'width': 640,
                'interpolation': 1,
            }],
            extra_arguments=[{'use_intra_process_comms': False}],
        ),
    ], [
        _stock_cvbridge_node(
            input_topic=_STOCK_CHAIN_RESIZE_OUT,
            output_topic='/legacy/image_processed',
            target_encoding='rgb8'),
    ]


def _stock_cvbridge_node(input_topic, output_topic, target_encoding):
    # Name `legacy_cvbridge` is what run.py greps for when summing
    # stock-side CPU/RSS — do not rename lightly.
    return Node(
        package='prism_image_proc',
        executable='cv_bridge_subscriber_node.py',
        name='legacy_cvbridge',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'input_topic': input_topic,
            'output_topic': output_topic,
            'target_encoding': target_encoding,
        }],
    )


# --------------------------------------------------------------------------
# Accel-side graphs (one per operation, all prism::*)
# --------------------------------------------------------------------------
def _accel_resize_node(video_path):
    return [
        _source_node('accel_source',
                     '/accelerated/image_raw', '/accelerated/camera_info',
                     video_path, True),
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
    ]


def _accel_crop_node(video_path):
    return [
        _source_node('accel_source',
                     '/accelerated/image_raw', '/accelerated/camera_info',
                     video_path, True),
        ComposableNode(
            package='prism_image_proc',
            plugin='prism::CropNode',
            name='accel_crop',
            parameters=[{
                'input_topic': '/accelerated/image_raw',
                'output_topic': '/accelerated/image_processed',
                'action': 'crop',
                'source_width': 3840,
                'source_height': 2160,
                'crop_x': 640,
                'crop_y': 360,
                'crop_width': 2560,
                'crop_height': 1440,
            }],
            extra_arguments=[{'use_intra_process_comms': True}],
        ),
    ]


def _accel_colorconvert_node(video_path):
    return [
        _source_node('accel_source',
                     '/accelerated/image_raw', '/accelerated/camera_info',
                     video_path, True),
        ComposableNode(
            package='prism_image_proc',
            plugin='prism::ColorConvertNode',
            name='accel_colorconvert',
            parameters=[{
                'input_topic': '/accelerated/image_raw',
                'output_topic': '/accelerated/image_processed',
                'action': 'colorconvert',
                'source_width': 3840,
                'source_height': 2160,
                'target_encoding': 'rgb8',
            }],
            extra_arguments=[{'use_intra_process_comms': True}],
        ),
    ]


def _accel_rectify_node(video_path):
    return [
        _source_node('accel_source',
                     '/accelerated/image_raw', '/accelerated/camera_info',
                     video_path, True,
                     extra_params=_RECTIFY_SOURCE_OVERRIDES),
        ComposableNode(
            package='prism_image_proc',
            plugin='prism::RectifyNode',
            name='accel_rectify',
            parameters=[{
                'input_topic': '/accelerated/image_raw',
                'output_topic': '/accelerated/image_processed',
                'action': 'rectify',
                'source_width': 3840,
                'source_height': 2160,
                'camera_info_input_topic': '/accelerated/camera_info',
                'camera_info_output_topic': '/accelerated/image_processed/camera_info',
            }],
            extra_arguments=[{'use_intra_process_comms': True}],
        ),
    ]


def _accel_chain_node(video_path):
    return [
        _source_node('accel_source',
                     '/accelerated/image_raw', '/accelerated/camera_info',
                     video_path, True),
        ComposableNode(
            package='prism_image_proc',
            plugin='prism::ImageProcNode',
            name='accel_chain',
            parameters=[{
                'input_topic': '/accelerated/image_raw',
                'output_topic': '/accelerated/image_processed',
                'action': 'crop,resize,colorconvert',
                'source_width': 3840,
                'source_height': 2160,
                'crop_x': 640,
                'crop_y': 360,
                'crop_width': 2560,
                'crop_height': 1440,
                'width': 640,
                'height': 480,
                'target_encoding': 'rgb8',
            }],
            extra_arguments=[{'use_intra_process_comms': True}],
        ),
    ]


# --------------------------------------------------------------------------
# Per-operation dispatch
# --------------------------------------------------------------------------
_STOCK_DISPATCH = {
    'resize':       _stock_resize_nodes,
    'crop':         _stock_crop_nodes,
    'colorconvert': _stock_colorconvert_nodes,
    'rectify':      _stock_rectify_nodes,
    'chain':        _stock_chain_nodes,
}

_ACCEL_DISPATCH = {
    'resize':       _accel_resize_node,
    'crop':         _accel_crop_node,
    'colorconvert': _accel_colorconvert_node,
    'rectify':      _accel_rectify_node,
    'chain':        _accel_chain_node,
}


def _build_nodes(context, *args, **kwargs):
    operation = LaunchConfiguration('operation').perform(context)
    video_path = LaunchConfiguration('video_path').perform(context)

    if operation not in _STOCK_DISPATCH:
        raise RuntimeError(
            f"unknown operation '{operation}'; expected one of "
            f"{sorted(_STOCK_DISPATCH)}")

    stock_composable, stock_standalone = _STOCK_DISPATCH[operation](video_path)
    accel_composable = _ACCEL_DISPATCH[operation](video_path)

    legacy_container = ComposableNodeContainer(
        name='legacy_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=stock_composable,
        output='screen',
        emulate_tty=True,
    )

    accel_container = ComposableNodeContainer(
        name='accel_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=accel_composable,
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

    return [legacy_container, accel_container, latency, *stock_standalone]


def generate_launch_description():
    # Disable Fast-DDS SHM transport: UDPv4-only profile avoids the
    # RTPS_TRANSPORT_SHM "Failed init_port" errors from stale /dev/shm locks.
    os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = _FASTDDS_PROFILE

    _reap_orphans()
    _purge_stale_fastdds_shm()

    return LaunchDescription([
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', _FASTDDS_PROFILE),
        DeclareLaunchArgument(
            'video_path', default_value='/tmp/test_video.mp4',
            description='4K source video piped into both A/B sides.'),
        DeclareLaunchArgument(
            'operation', default_value='resize',
            description='One of: resize, crop, colorconvert, chain. '
                        'See bench/operations.yaml for the A/B mapping.'),
        OpaqueFunction(function=_build_nodes),
    ])
