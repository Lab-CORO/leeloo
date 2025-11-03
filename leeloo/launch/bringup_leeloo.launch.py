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
    fusion_share = get_package_share_directory('pointcloud_fusion')

    # Launch file paths
    dsr_launch_file = os.path.join(dsr_bringup_share, 'launch', 'dsr_bringup2_rviz.launch.py')
    kinect_launch_file = os.path.join(kinect_driver_share, 'launch', 'driver.launch.py')
    fusion_launch_file = os.path.join(fusion_share, 'launch', 'pointcloud_fusion.launch.py')

    return LaunchDescription([
        # Launch arguments for Doosan
        DeclareLaunchArgument('mode', default_value='real'),
        DeclareLaunchArgument('host', default_value='192.168.50.100'),
        DeclareLaunchArgument('rt_host', default_value='192.168.50.50'),
        DeclareLaunchArgument('port', default_value='12345'),
        DeclareLaunchArgument('model', default_value='m1013'),


        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_base_link_to_camera_base',
            arguments=[
                '-0.1290', # x
                '-0.3955', # y 
                '0.8821',  # z
                '0.3666',   # yaw    (en radians)
                '0.7854',   # pitch (en radians)
                '0.2668',   # roll   (en radians)
                'base_link', #frame_id
                'camera_base' # child_frame_id
            ],
            output='log'
        ),

        # Include Doosan bringup launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(dsr_launch_file),
            launch_arguments={
                'mode': LaunchConfiguration('mode'),
                'host': LaunchConfiguration('host'),
                'port': LaunchConfiguration('port'),
                'model': LaunchConfiguration('model'),
            }.items()
        ),

        # Include Azure Kinect driver launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(kinect_launch_file)
        ),

        # Include Point Cloud Fusion launch file
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(fusion_launch_file)
        # ),
        Node(
            package='curobo_ros',
            executable='curobo_gen_traj',
            output='screen'
        ),

        # Launch robot segmentation node directly
        Node(
            package='curobo_ros',
            executable='robot_segmentation',
            name='robot_segmentation',
            output='screen',
            parameters=[
                {"joint_states_topic": "/dsr01/joint_states"},
                {"point_cloud_topic": "/points2" },
                {"robot_base_frame": "base_link" },
                {"robot_config_file": "/home/ros2_ws/src/curobo_ros/curobo_doosan/src/m1013/m1013.yml" },
                ]
        ),
    ])
