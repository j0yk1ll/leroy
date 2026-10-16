"""M16: launch real USB camera, rectification and DepthART ONLY.

Executed only inside the device-limited, network-none M16 capture image.
Does not create any motor-compatible control topic, XRSLAM, odometry,
map TF, synthetic sensors, or any autonomous motion process.
"""
import os
import stat

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


EXPECTED_ENV = {
    'LEROY_M16_SENSOR_ONLY': '1',
    'LEROY_NAV2_DRY_RUN': '1',
    'ROS_DOMAIN_ID': '168',
    'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST',
}


def _only_sensor_sources(context, *args, **kwargs):
    for key, expected in EXPECTED_ENV.items():
        if os.environ.get(key) != expected:
            raise RuntimeError(
                f'M16 camera-only launch requires {key}={expected}')
    video = LaunchConfiguration('video_device').perform(context).strip()
    info = LaunchConfiguration('camera_info_file').perform(context).strip()
    if not video.startswith('/dev/video'):
        raise RuntimeError('M16 only permits a /dev/videoN camera device')
    try:
        if not stat.S_ISCHR(os.stat(video).st_mode):
            raise RuntimeError('M16 video device is not a character device')
    except OSError as exc:
        raise RuntimeError('M16 camera node cannot access device') from exc
    if info != '/config/camera/front_camera.yaml' or not os.path.isfile(info):
        raise RuntimeError('M16 requires checked-in camera calibration file')
    depthart_config = os.path.join(
        get_package_share_directory('leroy_depthart'),
        'config', 'depthart.yaml')
    if not os.path.isfile(depthart_config):
        raise RuntimeError('M16 DepthART YAML not installed')
    # These are the EXACT optical parameters from existing robot.launch.py.
    camera = Node(
        package='usb_cam', executable='usb_cam_node_exe',
        namespace='camera', name='usb_cam', output='screen',
        parameters=[{
            'video_device': video,
            'image_width': 640, 'image_height': 480,
            'framerate': 30.0, 'io_method': 'mmap',
            'pixel_format': 'mjpeg2rgb',
            'camera_name': 'front_camera',
            'frame_id': 'camera_optical_frame',
            'camera_info_url': 'file://' + info,
        }],
    )
    rectifier = ComposableNodeContainer(
        package='rclcpp_components',
        executable='component_container',
        name='m16_image_proc_container',
        namespace='',
        output='screen',
        composable_node_descriptions=[
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='rectify_node',
                namespace='/camera',
                remappings=[
                    ('image', 'image_raw'),
                    ('camera_info', 'camera_info'),
                ],
            ),
        ],
    )
    depthart = Node(
        package='leroy_depthart', executable='depthart_node',
        name='depthart', output='screen',
        parameters=[depthart_config, {'device': 'cpu'}],
    )
    return [camera, rectifier, depthart]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('video_device', default_value='/dev/video2'),
        DeclareLaunchArgument(
            'camera_info_file',
            default_value='/config/camera/front_camera.yaml'),
        OpaqueFunction(function=_only_sensor_sources),
    ])
