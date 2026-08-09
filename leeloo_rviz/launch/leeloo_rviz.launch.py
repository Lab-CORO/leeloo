# RViz2 visualization + interaction session for the leeloo robot.
#
# This launch attaches to an ALREADY-RUNNING robot bringup
# (leeloo/launch/bringup_leeloo.launch.py). It starts no drivers. It:
#   1. Opens RViz2 with the leeloo config, with the node's /tf and /tf_static
#      remapped onto the unified /leeloo/tf[_static] tree (the only topics that
#      carry the full arm + base + camera transforms).
#   2. Optionally launches a "ghost" preview robot: an arm-only robot_state_publisher
#      (frame_prefix preview/) driven by the trajectory_preview panel's
#      /trajectory/joint_states, grafted onto the real arm at dsr01/world so a
#      planned trajectory can be scrubbed/replayed as a translucent overlay.
#
# The ghost's /tf[_static] are also remapped onto /leeloo/tf[_static] so RViz
# (reading /leeloo/tf) sees the preview/-prefixed frames without a separate relay.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    variant = LaunchConfiguration('variant')
    rviz_config = LaunchConfiguration('rviz_config')
    launch_preview = LaunchConfiguration('launch_preview')

    declare_variant = DeclareLaunchArgument(
        'variant',
        default_value='view',
        choices=['view', 'curobo'],
        description=(
            "Which shipped config to load when rviz_config is not overridden: "
            "'view' = robot + cameras + trajectory preview (default); "
            "'curobo' = cuRobo setup panel + collision spheres, NO cameras "
            "(CuRoboSetupPanel and camera Image displays cannot coexist)."
        ),
    )

    # rviz_config defaults to the file matching `variant`; pass rviz_config
    # explicitly to load an arbitrary config.
    default_config = PathJoinSubstitution([
        FindPackageShare('leeloo_rviz'),
        'rviz',
        PythonExpression(
            ["'leeloo_curobo.rviz' if '", variant, "' == 'curobo' else 'leeloo.rviz'"]
        ),
    ])

    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_config,
        description='Path to the RViz2 .rviz config to load (overrides `variant`).',
    )

    declare_launch_preview = DeclareLaunchArgument(
        'launch_preview',
        default_value='true',
        description='Launch the ghost/preview arm pipeline for trajectory replay.',
    )

    # Unify RViz onto the /leeloo TF tree.
    tf_remaps = [
        ('/tf', '/leeloo/tf'),
        ('/tf_static', '/leeloo/tf_static'),
    ]

    # Unique node name so we don't collide with the robot bringup's own `rviz2`
    # (two nodes sharing a name breaks parameter service calls).
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='leeloo_rviz',
        arguments=['-d', rviz_config],
        remappings=tf_remaps,
        output='screen',
    )

    # Arm-only URDF for the ghost. IMPORTANT: this must use the joint names the
    # planner puts on /trajectory, which are joint1..joint6 (NO underscore) with
    # links base_0/link1..6 — the urdf_visualizer m1013 flavour, NOT the
    # dsr_description2 one (joint_1..6). Otherwise the preview joint_state_publisher
    # silently drops every trajectory sample and the ghost never moves. Meshes
    # resolve via package://urdf_visualizer (installed). frame_prefix preview/
    # turns the frames into preview/base_0, preview/link1..6.
    ghost_description = Command(
        ['xacro ', PathJoinSubstitution(
            [FindPackageShare('leeloo_rviz'), 'urdf', 'm1013_ghost.urdf']
        )]
    )

    preview_pipeline = GroupAction(
        condition=IfCondition(launch_preview),
        actions=[
            # Fold the panel's sampled /trajectory/joint_states into a continuous
            # joint state stream for the ghost RSP.
            Node(
                package='joint_state_publisher',
                executable='joint_state_publisher',
                name='joint_state_publisher',
                namespace='preview',
                parameters=[{
                    'source_list': ['/trajectory/joint_states'],
                }],
                remappings=tf_remaps,
                output='screen',
            ),
            # Ghost arm; publishes /preview/robot_description + preview/-prefixed TF.
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                namespace='preview',
                parameters=[{
                    'robot_description': ghost_description,
                    'frame_prefix': 'preview/',
                }],
                remappings=tf_remaps,
                output='screen',
            ),
            # Graft the ghost onto the real arm base.
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='preview_graft',
                namespace='preview',
                arguments=[
                    '--x', '0', '--y', '0', '--z', '0',
                    '--roll', '0', '--pitch', '0', '--yaw', '0',
                    '--frame-id', 'dsr01/base_link',
                    '--child-frame-id', 'preview/base_0',
                ],
                remappings=tf_remaps,
                output='screen',
            ),
        ],
    )

    return LaunchDescription([
        declare_variant,
        declare_rviz_config,
        declare_launch_preview,
        rviz_node,
        preview_pipeline,
    ])
