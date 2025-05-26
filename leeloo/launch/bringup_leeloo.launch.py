# from launch import LaunchDescription
# from launch_ros.actions import Node

# def generate_launch_description():
#     return LaunchDescription([
#         # Launch doosan robot
#         # ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=real host:=192.168.137.100 port:=12345 model:=m1013

#         # Launch azure kinect camera
#         # ros2 launch  azure_kinect_ros_driver driver.launch.py 

#         # Launch point cloud fusion
#         # ros2 launch  pointcloud_fusion pointcloud_fusion.launch.py

#         # Launch robot segmentation
#         # ros2 run curobo_ros robot_segmentation
#     ])


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
        DeclareLaunchArgument('host', default_value='192.168.137.100'),
        DeclareLaunchArgument('port', default_value='12345'),
        DeclareLaunchArgument('model', default_value='m1013'),

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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(fusion_launch_file)
        ),

        # Launch robot segmentation node directly
        Node(
            package='curobo_ros',
            executable='robot_segmentation',
            name='robot_segmentation',
            output='screen',
        ),
    ])
