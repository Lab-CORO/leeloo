from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package share paths
    dsr_bringup_share = get_package_share_directory('dsr_bringup2')
    kinect_driver_share = get_package_share_directory('azure_kinect_ros_driver')
    curobo_share = get_package_share_directory('curobo_ros')
    fusion_share = get_package_share_directory('pointcloud_fusion')
    handeye_share = get_package_share_directory('hand_eye_calibration')

    # Launch file paths
    dsr_launch_file = os.path.join(dsr_bringup_share, 'launch', 'dsr_bringup2_rviz.launch.py')
    kinect_launch_file = os.path.join(kinect_driver_share, 'launch', 'driver.launch.py')
    fusion_launch_file = os.path.join(fusion_share, 'launch', 'pointcloud_fusion.launch.py')
    curobo_launch_file = os.path.join(curobo_share, 'launch', 'gen_traj.launch.py')
    handeye_launch_file = os.path.join(handeye_share, 'calibration.launch.py')
    return LaunchDescription([
        # Launch arguments for Doosan
        DeclareLaunchArgument('mode', default_value='real'),
        DeclareLaunchArgument('host', default_value='192.168.50.100'),
        DeclareLaunchArgument('rt_host', default_value='192.168.50.50'),
        DeclareLaunchArgument('port', default_value='12345'),
        DeclareLaunchArgument('model', default_value='m1013'),
        DeclareLaunchArgument('gui', default_value='False'),


        # Include Doosan bringup launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(dsr_launch_file),
            launch_arguments={
                'mode': LaunchConfiguration('mode'),
                'host': LaunchConfiguration('host'),
                'port': LaunchConfiguration('port'),
                'model': LaunchConfiguration('model'),
                'gui': LaunchConfiguration('gui'),
            }.items()
        ),

        # Include Azure Kinect driver launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(kinect_launch_file)
        ),

        Node(
            package='leeloo',
            executable='execute_trajectory',
            output='screen'
        ),

        # Détection du marqueur ArUco (fournit le frame TF 'marker')
        Node(
            package='ros2_markertracker',
            executable='markertracker_node',
            output='screen',
            namespace='/ros2_markertracker',
            parameters=[{
                'input_image_topic': '/rgb/image_raw',
                'publish_topic_image_result': True,
                'camera_info_topic': '/rgb/camera_info',
                'marker_length': 8.6,
                'aruco_dictionary_id': 'DICT_APRILTAG_36H11',
                'camera_frame_id': 'rgb_camera_link',
                'marker_frame_id': 'marker',
                'ignore_marker_ids_array': 17,
            }]
        ),

        # Service de capture hand-eye (/hand_eye_calibration/capture_point)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(handeye_launch_file),
            launch_arguments={
                'calibration_type': 'eye-on-base',
                'namespace_prefix': 'kinect_calib',
                'freehand_robot_movement': 'true',
                'robot_base_frame': 'base_link',
                'robot_effector_frame': 'link_6',
                'tracking_base_frame': 'rgb_camera_link',
                'tracking_marker_frame': 'marker',
            }.items()
        ),

        Node(
            package='leeloo_calibration',
            executable='kinect_tf_computation_node',
            name='kinect_tf_computation_node',
            output='screen',
        ),

        Node(
            package='leeloo_calibration',
            executable='auto_calibration_server',
            name='auto_calibration_server',
            output='screen',
        ),

    ])
