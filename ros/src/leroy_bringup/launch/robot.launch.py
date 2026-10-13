from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_camera = LaunchConfiguration('use_camera')
    use_depthart = LaunchConfiguration('use_depthart')

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

    # DepthART's exact executable/parameter names are installation-specific.
    # We invoke the package's own launch file so its existing tested config remains authoritative.
    depthart = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('depthart_ros'),
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
        description,
        bridge,
        camera,
        image_proc_container,
        depthart,
    ])
