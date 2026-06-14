from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package share paths
    dsr_bringup_share   = get_package_share_directory('dsr_bringup2')
    kinect_driver_share = get_package_share_directory('azure_kinect_ros_driver')
    curobo_ros_share = get_package_share_directory('curobo_ros')
    handeye_share       = get_package_share_directory('hand_eye_calibration')

    # Launch file paths
    dsr_launch_file        = os.path.join(dsr_bringup_share,    'launch', 'dsr_bringup2_rviz.launch.py')
    curobo_ros_launch_file = os.path.join(curobo_ros_share,     'launch', 'gen_traj.launch.py')
    kinect_launch_file     = os.path.join(kinect_driver_share,  'launch', 'driver.launch.py')
    handeye_launch_file    = os.path.join(handeye_share,        'calibration.launch.py')

    return LaunchDescription([
        # ── Fix "sequence size exceeds remaining buffer" ───────────────────────
        # FastDDS floods its discovery buffer when N nodes start simultaneously.
        # Raising the history depth prevents message loss during startup.
        SetEnvironmentVariable(
            'RMW_FASTRTPS_USE_QOS_FROM_XML', '0'
        ),
        SetEnvironmentVariable(
            'FASTRTPS_DEFAULT_PROFILES_FILE', ''
        ),

        # ── k4a GPU depth engine ───────────────────────────────────────────────
        # libdepthengine.so uses OpenGL compute shaders for depth processing.
        # GLVND defaults to Mesa (llvmpipe, CPU) via Xvfb's vendor advertisement.
        # Forcing the NVIDIA vendor makes the depth engine run on the Tegra GPU.
        SetEnvironmentVariable('DISPLAY', ':99'),
        SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia'),

        # ── Launch arguments ──────────────────────────────────────────────────
        DeclareLaunchArgument('mode',     default_value='real'),
        DeclareLaunchArgument('host',     default_value='192.168.50.100'),
        DeclareLaunchArgument('rt_host',  default_value='192.168.50.50'),
        DeclareLaunchArgument('port',     default_value='12345'),
        DeclareLaunchArgument('model',    default_value='m1013'),
        DeclareLaunchArgument('gui',      default_value='False'),
        
        
        DeclareLaunchArgument('cameras_config_file',    default_value='/home/ros2_ws/src/leeloo/config/cameras.yaml'),
        DeclareLaunchArgument('world_file',             default_value='/home/ros2_ws/src/leeloo/config/leeloo_world.yaml'),
        DeclareLaunchArgument('voxel_size',             default_value='0.02'),


        # Kinect camera arguments — tunable without editing this file.
        # NFOV_UNBINNED uses ~40% less USB bandwidth than WFOV_UNBINNED,
        # which prevents the "Failed to poll cameras" FATAL crash.
        #
        # color_format: 'jpeg' passes the hardware-compressed MJPEG bytes straight
        #   through — zero CPU decode in the k4a SDK. The color topic becomes
        #   /rgb/image_raw/compressed instead of /rgb/image_raw.
        #   For calibration (markertracker needs the raw topic), launch with
        #   color_format:=bgra.
        DeclareLaunchArgument('depth_mode',        default_value='NFOV_UNBINNED'),
        DeclareLaunchArgument('color_enabled',     default_value='true'),
        DeclareLaunchArgument('color_format',      default_value='bgra'),
        DeclareLaunchArgument('color_resolution',  default_value='720P'),
        DeclareLaunchArgument('fps',               default_value='5'),
        DeclareLaunchArgument('rgb_point_cloud',   default_value='false'),
        DeclareLaunchArgument('point_cloud',       default_value='false'),
        DeclareLaunchArgument('imu_rate_target',   default_value='100'),

        # ── Doosan bringup ────────────────────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(dsr_launch_file),
            launch_arguments={
                'mode':    LaunchConfiguration('mode'),
                'host':    LaunchConfiguration('host'),
                'port':    LaunchConfiguration('port'),
                'model':   LaunchConfiguration('model'),
                'gui':     LaunchConfiguration('gui'),
            }.items()
        ),

        # ── Azure Kinect ──────────────────────────────────────────────────────
        # Pass bandwidth-safe defaults via launch_arguments.
        # driver.launch.py handles its own robot_state_publisher internally.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(kinect_launch_file),
            launch_arguments={
                'depth_mode':       LaunchConfiguration('depth_mode'),
                'color_enabled':    LaunchConfiguration('color_enabled'),
                'color_format':     LaunchConfiguration('color_format'),
                'color_resolution': LaunchConfiguration('color_resolution'),
                'fps':              LaunchConfiguration('fps'),
                'rgb_point_cloud':  LaunchConfiguration('rgb_point_cloud'),
                'point_cloud':      LaunchConfiguration('point_cloud'),
                'imu_rate_target':  LaunchConfiguration('imu_rate_target'),
            }.items()
        ),

        #── Curobo Ros ────────────────────────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(curobo_ros_launch_file),
            launch_arguments={
                'gui': LaunchConfiguration('gui'),
                # 'cameras_config_file': LaunchConfiguration('cameras_config_file'),
                'world_file': LaunchConfiguration('world_file'),
                'voxel_size': LaunchConfiguration('voxel_size'),

            }.items()
        ),
        Node(
            package='curobo_ros',
            executable='robot_segmentation',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[{
                'robot_base_frame':  'base_0',
            }]

        ),



        # ── Core nodes ────────────────────────────────────────────────────────
        Node(
            package='leeloo',
            executable='execute_trajectory',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
        ),

        # Node(
        #     package='curobo_ros',
        #     executable='curobo_trajectory_planner',
        #     output='screen',
        # ),

        # ── Calibration nodes (delayed: wait for camera TF to be available) ──
        # markertracker_node and hand_eye_calibration crash on import because
        # ros2_markertracker_interfaces is not built in the container.
        # TODO: add ros2_markertracker_interfaces to the workspace build,
        # then remove the delay — it does not fix the missing module, only
        # prevents the crash from blocking other nodes at startup.
        TimerAction(period=5.0, actions=[
            Node(
                package='ros2_markertracker',
                executable='markertracker_node',
                output='screen',
                namespace='/ros2_markertracker',
                respawn=True,
                respawn_delay=5.0,
                parameters=[{
                    'input_image_topic':          '/rgb/image_raw',
                    'publish_topic_image_result': True,
                    'camera_info_topic':          '/rgb/camera_info',
                    'marker_length':              8.6,
                    'aruco_dictionary_id':        'DICT_APRILTAG_36H11',
                    'camera_frame_id':            'rgb_camera_link',
                    'marker_frame_id':            'marker',
                    'ignore_marker_ids_array':    17,
                }]
            ),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(handeye_launch_file),
                launch_arguments={
                    'calibration_type':        'eye-on-base',
                    'namespace_prefix':        'kinect_calib',
                    'freehand_robot_movement': 'true',
                    'robot_base_frame':        'base_link',
                    'robot_effector_frame':    'link_6',
                    'tracking_base_frame':     'rgb_camera_link',
                    'tracking_marker_frame':   'marker',
                }.items()
            ),

            Node(
                package='leeloo_calibration',
                executable='auto_calibration_server',
                name='auto_calibration_server',
                output='screen',
                respawn=True,
                respawn_delay=5.0,
            ),
        ]),

        # kinect_tf_computation_node is independent of the interfaces issue
        Node(
            package='leeloo_calibration',
            executable='kinect_tf_computation_node',
            name='kinect_tf_computation_node',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[{
                'calibration_result_file': '/home/ros2_ws/src/leeloo_calibration/config/kinect_hand_eye_result.yaml',
            }]

        ),
    ])
