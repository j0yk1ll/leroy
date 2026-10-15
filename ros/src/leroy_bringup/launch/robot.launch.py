import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def _launch_xrslam(context, *args, **kwargs):
    """Start the Leroy-owned XRSLAM ROS 2 adapter when explicitly enabled."""
    def arg(name):
        return LaunchConfiguration(name).perform(context).strip()

    required = (
        'xrslam_slam_yaml',
        'xrslam_device_template_yaml',
        'xrslam_image_topic',
        'xrslam_camera_info_topic',
    )
    missing = [name for name in required if not arg(name)]
    if missing:
        raise RuntimeError(
            'use_xrslam:=true requires explicit calibrated inputs: '
            + ', '.join(missing)
        )

    # The adapter reads sensor.mode from XRSLAM SLAM YAML. An offset is
    # mandatory for monocular_imu and optional for pure monocular; the adapter
    # enforces the mode-specific rule and rejects visual-only metric odometry.
    offset_text = arg('xrslam_camera_imu_offset_sec')
    offset = None
    if offset_text:
        try:
            offset = float(offset_text)
        except ValueError as exc:
            raise RuntimeError('xrslam_camera_imu_offset_sec must be a number') from exc
        if not math.isfinite(offset):
            raise RuntimeError('xrslam_camera_imu_offset_sec must be finite')

    pose_validated = arg('xrslam_pose_convention_validated').lower()
    if pose_validated not in ('true', 'false'):
        raise RuntimeError('xrslam_pose_convention_validated must be true or false')

    xrslam_params = {
            'slam_yaml': arg('xrslam_slam_yaml'),
            'device_template_yaml': arg('xrslam_device_template_yaml'),
            'image_topic': arg('xrslam_image_topic'),
            'camera_info_topic': arg('xrslam_camera_info_topic'),
            'imu_topic': arg('xrslam_imu_topic'),
            'base_frame': 'base_link',
            'imu_frame': 'imu_link',
            'camera_optical_frame': arg('camera_frame'),
            'odom_frame': 'odom',
            'pose_convention_validated': pose_validated == 'true',
            # No map->odom until persistent map alignment is verified.
            'enable_map_tf': False,
    }
    if offset is not None:
        xrslam_params['camera_imu_offset_sec'] = offset

    return [Node(
        package='leroy_xrslam',
        executable='leroy_xrslam_node',
        name='leroy_xrslam',
        output='screen',
        parameters=[xrslam_params],
    )]


def generate_launch_description():
    use_camera = LaunchConfiguration('use_camera')
    use_depthart = LaunchConfiguration('use_depthart')
    use_xrslam = LaunchConfiguration('use_xrslam')

    video_device = LaunchConfiguration('video_device')
    camera_name = LaunchConfiguration('camera_name')
    camera_frame = LaunchConfiguration('camera_frame')
    camera_info_file = LaunchConfiguration('camera_info_file')
    image_width = LaunchConfiguration('image_width')
    image_height = LaunchConfiguration('image_height')
    framerate = LaunchConfiguration('framerate')
    io_method = LaunchConfiguration('io_method')
    pixel_format = LaunchConfiguration('pixel_format')

    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('leroy_description'),
                'launch',
                'description.launch.py',
            ])
        )
    )

    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('leroy_bridge'),
                'launch',
                'leroy_bridge.launch.py',
            ])
        )
    )

    camera = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        namespace='camera',
        name='usb_cam',
        output='screen',
        condition=IfCondition(use_camera),
        parameters=[{
            'video_device': video_device,
            'image_width': image_width,
            'image_height': image_height,
            'framerate': framerate,
            'io_method': io_method,
            'pixel_format': pixel_format,
            'camera_name': camera_name,
            'frame_id': camera_frame,
            'camera_info_url': ['file://', camera_info_file],
        }],
    )

    image_proc_container = ComposableNodeContainer(
        package='rclcpp_components',
        executable='component_container',
        name='image_proc_container',
        namespace='',
        output='screen',
        condition=IfCondition(use_camera),
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

    # DepthART stays independent: no depth data is fed into localization or
    # safety-critical collision avoidance by this launch.
    depthart = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('leroy_depthart'),
                'launch',
                'depthart.launch.py',
            ])
        ),
        condition=IfCondition(use_depthart),
    )

    return LaunchDescription([
        DeclareLaunchArgument('video_device', default_value='/dev/video2'),
        DeclareLaunchArgument('camera_name', default_value='front_camera'),
        DeclareLaunchArgument('camera_frame', default_value='camera_optical_frame'),
        DeclareLaunchArgument(
            'camera_info_file',
            default_value='/config/camera/front_camera.yaml',
        ),
        DeclareLaunchArgument('image_width', default_value='640'),
        DeclareLaunchArgument('image_height', default_value='480'),
        DeclareLaunchArgument('framerate', default_value='30.0'),
        DeclareLaunchArgument('io_method', default_value='mmap'),
        DeclareLaunchArgument('pixel_format', default_value='mjpeg2rgb'),
        DeclareLaunchArgument('use_camera', default_value='true'),
        DeclareLaunchArgument('use_depthart', default_value='true'),
        DeclareLaunchArgument('use_xrslam', default_value='false'),
        # No guesswork: find actual rectified topic + matching CameraInfo.
        DeclareLaunchArgument('xrslam_image_topic', default_value=''),
        DeclareLaunchArgument('xrslam_camera_info_topic', default_value=''),
        DeclareLaunchArgument('xrslam_imu_topic', default_value='/imu/data_raw'),
        # Managed as calibration assets by the Leroy deployment.
        DeclareLaunchArgument('xrslam_slam_yaml', default_value=''),
        DeclareLaunchArgument('xrslam_device_template_yaml', default_value=''),
        # Required for monocular_imu only; omit for visual-only diagnostics.
        DeclareLaunchArgument('xrslam_camera_imu_offset_sec', default_value=''),
        DeclareLaunchArgument(
            'xrslam_pose_convention_validated', default_value='false',
        ),
        description,
        bridge,
        camera,
        image_proc_container,
        depthart,
        OpaqueFunction(function=_launch_xrslam, condition=IfCondition(use_xrslam)),
    ])
