from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PythonExpression
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue
import os
import shutil
import tempfile
import yaml

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    dsr_bringup_share = get_package_share_directory('dsr_bringup2')
    leeloo_share       = get_package_share_directory('leeloo')
    curobo_ros_share   = get_package_share_directory('curobo_ros')

    dsr_launch_file = os.path.join(dsr_bringup_share, 'launch', 'dsr_bringup2_rviz.launch.py')

    # CuRobo config — read base_link and urdf_path from the YAML so they stay
    # consistent with what the planner reads at runtime (single source of truth).
    #
    # robot_config_file is handed to cuRobo as a bare path (config_wrapper_motion.py
    # passes ConfigManager.robot_config_file straight to MotionPlannerCfg.create),
    # so cuRobo parses urdf_path/asset_root_path itself — ConfigManager's own
    # load_yaml() of this file is never consulted for those fields. The checked-in
    # leeloo_curobo.yaml therefore still carries the old hardcoded
    # /home/ros2_ws/src/... values (they need to resolve for a standalone cuRobo
    # load outside this launch file too). Rewrite them here to the install-space
    # share/ layout, same pattern as the sim-only XML injection in
    # leeloo_gazebo.launch.py, and hand cuRobo the patched copy instead of editing
    # the source YAML.
    leeloo_curobo_config_src = os.path.join(leeloo_share, 'config', 'leeloo_curobo.yaml')
    leeloo_urdf = os.path.join(leeloo_share, 'urdf', 'leeloo.urdf')
    with open(leeloo_curobo_config_src, 'r') as f:
        _cfg = yaml.safe_load(f)
    _kin = _cfg.get('robot_cfg', {}).get('kinematics', {})
    curobo_base_link = _kin.get('base_link', 'base_link')  # noqa: F841 (documente la cohérence attendue avec base_link ci-dessous)
    # asset_root_path/leeloo/urdf/leeloo.urdf must still equal urdf_path after the
    # rewrite (meshes are referenced relative to asset_root_path); dirname(leeloo_share)
    # + '/leeloo/...' reduces back to leeloo_share + '/...', so this preserves the
    # exact relative layout the checked-in values encoded.
    if 'urdf_path' in _kin:
        _kin['urdf_path'] = leeloo_urdf
    if 'asset_root_path' in _kin:
        _kin['asset_root_path'] = os.path.dirname(leeloo_share)
    curobo_urdf_path = _kin.get('urdf_path', leeloo_urdf)  # noqa: F841

    _patched_curobo_config = tempfile.NamedTemporaryFile(
        mode='w', suffix='_leeloo_curobo.yaml', delete=False)
    yaml.safe_dump(_cfg, _patched_curobo_config)
    _patched_curobo_config.close()
    leeloo_curobo_config = _patched_curobo_config.name

    # Gripper Robotiq 2F-85 : ros2_control_node a besoin du <ros2_control> ajouté
    # dans leeloo.urdf (driver réel, COM_port /dev/ttyUSB0) → on lui passe le même
    # robot_description que leeloo_viz_state_publisher, pas de fichier séparé.
    robotiq_description_share = get_package_share_directory('robotiq_description')
    gripper_controllers_file = os.path.join(robotiq_description_share, 'config', 'robotiq_controllers.yaml')
    gripper_update_rate_file = os.path.join(robotiq_description_share, 'config', 'robotiq_update_rate.yaml')

    # ── Debug harness for the CuRobo planner (SIGSEGV / CUDA fault capture) ─────
    # Select with the CUROBO_DEBUG env var at launch time:
    #   CUROBO_DEBUG=gdb       → native C++/CUDA backtrace on crash (bt + threads)
    #   CUROBO_DEBUG=memcheck  → compute-sanitizer memcheck: exact out-of-bounds
    #                            kernel + address. Slow warmup — reproduce the
    #                            crash on demand, only this node is wrapped.
    #   (unset / none)         → normal run (default)
    # PYTHONFAULTHANDLER is ALWAYS on: it prints the Python traceback even on a
    # native segfault (exit -11), at near-zero cost — so a bare crash is no longer
    # silent. Under gdb/memcheck respawn is disabled so the crash is caught once
    # instead of being immediately restarted.
    _curobo_debug = os.environ.get('CUROBO_DEBUG', 'none').lower()
    # Resolve the wrapper binary. Prefer cuda-gdb (ships with CUDA, gives device
    # backtraces of the faulting kernel) over plain gdb. A missing tool must NOT
    # take down the whole launch — warn and run the planner normally instead.
    _gdb = shutil.which('cuda-gdb') or shutil.which('gdb')
    _memcheck = shutil.which('compute-sanitizer')
    # The ROS 2 "executable" is a #!/usr/bin/python3 entry-point *script*, not an
    # ELF binary — so the debug tool must run the interpreter with the script as
    # its argument (else cuda-gdb: "not in executable format"). launch appends the
    # script path + args after this prefix, giving e.g.
    #   cuda-gdb … --args /usr/bin/python3 <script> --ros-args …
    _curobo_prefix = {
        'gdb': (f'{_gdb} -batch -ex run -ex bt -ex "thread apply all bt" '
                f'--args /usr/bin/python3' if _gdb else None),
        'memcheck': (f'{_memcheck} --tool memcheck --destroy-on-device-error kernel '
                     f'--launch-timeout 900 /usr/bin/python3' if _memcheck else None),
    }.get(_curobo_debug)  # None for 'none' / 'faulthandler' / missing tool
    if _curobo_debug in ('gdb', 'memcheck') and _curobo_prefix is None:
        print(f"[control.launch] CUROBO_DEBUG={_curobo_debug} requested but the tool "
              f"was not found — running the planner normally. "
              f"(install gdb via `apt-get install gdb`; compute-sanitizer ships with CUDA.)")

    # Env for the planner node. faulthandler is always on. In gdb mode, prepend the
    # Tegra driver dir so cuda-gdb can dlopen its device backend
    # (libcudadebugger.so.1) — it lives there but is NOT in the ldconfig cache, so
    # without this cuda-gdb reports "Could not find CUDA Debugger back-end". Prepend
    # (not replace) to keep the ROS LD_LIBRARY_PATH intact.
    _curobo_env = {'PYTHONFAULTHANDLER': '1'}
    if _curobo_debug == 'gdb' and _gdb:
        _curobo_env['LD_LIBRARY_PATH'] = [
            '/usr/lib/aarch64-linux-gnu/tegra:',
            EnvironmentVariable('LD_LIBRARY_PATH', default_value=''),
        ]

    # ── Mode simulation ────────────────────────────────────────────────────────
    # sim:=true débranche uniquement le MATÉRIEL : dsr_bringup2 (driver Doosan)
    # et le ros2_control_node de la pince réelle (RobotiqGripperHardwareInterface
    # sur /dev/ttyUSB0). Tout le reste — curobo, robot_segmentation,
    # execute_trajectory, ridgeback_control — tourne à l'identique : leeloo_gazebo
    # expose les mêmes topics et types que ces drivers, y compris
    # /dsr01/dsr_controller2/speedj_rt_stream et /dsr01/joint_states.
    _not_sim = IfCondition(PythonExpression(
        ["not '", LaunchConfiguration('sim'), "'.lower() in ('true', '1')"]))
    # Horloge simulée. Ne rend PAS la chaîne d'exécution sim-time-paced : la
    # boucle servo MPC, JointSpeedStrategy et le wall timer d'execute_trajectory
    # restent cadencés sur l'horloge murale. Uniquement là pour que les
    # timestamps TF/images soient cohérents avec /clock (sans quoi tout lookup
    # TF échoue, le temps sim partant de ~0 face à un temps mur de ~1.7e9).
    _use_sim_time = ParameterValue(LaunchConfiguration('sim'), value_type=bool)

    return LaunchDescription([
        DeclareLaunchArgument(
            'sim', default_value='false',
            description="true : matériel simulé par leeloo_gazebo (driver Doosan "
                        "et pince réelle débranchés)."),

        DeclareLaunchArgument('tf_remap',        default_value='/leeloo/tf'),
        DeclareLaunchArgument('tf_static_remap', default_value='/leeloo/tf_static'),

        DeclareLaunchArgument('mode',     default_value='real'),
        DeclareLaunchArgument('host',     default_value='192.168.50.100'),
        DeclareLaunchArgument('rt_host',  default_value='192.168.50.50'),
        DeclareLaunchArgument('port',     default_value='12345'),
        DeclareLaunchArgument('model',    default_value='m1013'),
        DeclareLaunchArgument('gui',      default_value='False'),

        DeclareLaunchArgument('cameras_config_file', default_value=os.path.join(leeloo_share, 'config', 'cameras.yaml')),
        DeclareLaunchArgument('world_file',          default_value=os.path.join(leeloo_share, 'config', 'leeloo_world.yaml')),
        DeclareLaunchArgument('voxel_size',          default_value='0.02'),

        # MPC diagnostic CSVs (mpc_diag_*.csv + speedj_publish_*.csv), off by
        # default. Passed as launch args rather than set with `ros2 param set`
        # because both toggles are read at goal setup: a relaunch resets them to
        # their declared defaults, so a run started right after one silently
        # produced no CSV at all and the analysis then re-read the PREVIOUS run's
        # file. Launch with mpc_debug:=true to keep them across restarts.
        DeclareLaunchArgument('mpc_debug', default_value='false'),
        DeclareLaunchArgument('diag_dir',  default_value='/home/ros2_ws/diag'),
        DeclareLaunchArgument('diag_dir',  default_value='/home/ros2_ws/diag'),

        # MPC cost/optimizer config, as editable YAML instead of values baked
        # into mpc_planner.py -- see curobo_ros/config/mpc/{mppi,lbfgs}_mpc.yaml
        # for the shipped defaults and their tuning history. Override with e.g.
        # `ros2 launch leeloo bringup_leeloo.launch.py mpc_lbfgs_config_file:=/path/to/custom.yaml`.
        DeclareLaunchArgument(
            'mpc_mppi_config_file',
            default_value=os.path.join(curobo_ros_share, 'config', 'mpc', 'mppi_mpc.yaml')),
        DeclareLaunchArgument(
            'mpc_lbfgs_config_file',
            default_value=os.path.join(curobo_ros_share, 'config', 'mpc', 'lbfgs_reactive.yaml')),

        # Outgoing accel clamp override (JointSpeedStrategy._clamp_velocities,
        # deg/s^2). Referenced below but was never declared, which raised at
        # launch resolution ("launch configuration 'max_joint_accel_dps' does
        # not exist"). Default matches the m1013 descriptor's own default (45)
        # so an unset launch arg changes nothing. Also live-settable without a
        # relaunch via `ros2 param set <node> max_joint_accel_dps <value>`.
        DeclareLaunchArgument('max_joint_accel_dps', default_value='1000.0'),

        # ── Doosan bringup (scope global : ne pas casser les internes dsr01) ───
        # En sim : remplacé par dsr_sim_bridge (leeloo_gazebo), qui consomme le
        # même /dsr01/dsr_controller2/speedj_rt_stream et publie le même
        # /dsr01/joint_states. execute_trajectory n'appelle aucun service Doosan,
        # donc il n'a besoin de rien d'autre.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(dsr_launch_file),
            condition=_not_sim,
            launch_arguments={
                'mode':  LaunchConfiguration('mode'),
                'host':  LaunchConfiguration('host'),
                'port':  LaunchConfiguration('port'),
                'model': LaunchConfiguration('model'),
                'gui':   LaunchConfiguration('gui'),
                # Préfixe TF sur le RSP Doosan → frames dsr01/world, dsr01/base_link,
                # dsr01/link_1…6. Évite la collision base_link avec le Ridgeback et
                # rend la racine du bras identifiable de façon unique.
                'frame_prefix': 'dsr01/',
            }.items()
        ),

        GroupAction([
            SetRemap('/tf',        LaunchConfiguration('tf_remap')),
            SetRemap('/tf_static', LaunchConfiguration('tf_static_remap')),

            # Pince réelle. En sim, gz_ros2_control héberge son propre
            # controller_manager et sert un robotiq_gripper_controller de même
            # nom et même type (parallel_gripper_action_controller/
            # GripperActionController), donc /robotiq_gripper_controller/gripper_cmd
            # existe dans les deux cas. Lancer celui-ci en plus créerait un
            # second /controller_manager en collision sur le graphe.
            Node(
                package='controller_manager',
                executable='ros2_control_node',
                output='screen',
                condition=_not_sim,
                parameters=[
                    gripper_update_rate_file,
                    gripper_controllers_file,
                ],
                remappings=[
                    ('robot_description', '/leeloo/robot_description'),
                ],
            ),
            Node(
                package='controller_manager',
                executable='spawner',
                condition=_not_sim,
                arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
            ),
            Node(
                package='controller_manager',
                executable='spawner',
                condition=_not_sim,
                arguments=['robotiq_gripper_controller', '-c', '/controller_manager'],
            ),
            Node(
                package='controller_manager',
                executable='spawner',
                condition=_not_sim,
                arguments=['robotiq_activation_controller', '-c', '/controller_manager'],
            ),

            Node(
                package='curobo_ros',
                executable='curobo_trajectory_planner',
                name='curobo_trajectory_planner',
                output='screen',
                # Debug harness — controlled by CUROBO_DEBUG (see top of this function).
                prefix=_curobo_prefix,
                additional_env=_curobo_env,
                respawn=(_curobo_prefix is None),  # no respawn under gdb/memcheck
                respawn_delay=5.0,
                parameters=[{
                    'robot_config_file': leeloo_curobo_config,
                    'urdf_path': leeloo_urdf,
                    'planner_type': 'classic',
                    'interpolation_dt': 0.03,
                    'cameras_config_file': LaunchConfiguration('cameras_config_file'),
                    'base_link': "dsr01/world",
                    'world_file': LaunchConfiguration('world_file'),
                    'voxel_size': LaunchConfiguration('voxel_size'),
                    'mapper_extent_xyz': [2.56, 2.56, 2.56],
                    'mapper_grid_center': [0.0, 0.0, 0.0],
                    'mapper_image_width': 1280,
                    'mapper_image_height': 720,
                    'mpc_solver_type': 'lbfgs_bspline',#mppi_acceleration,lbfgs_bspline
                    # 5, not 10: doubling this (test 2026-08-07) doubled solve_ms
                    # from 105ms to 199ms median against a 240ms
                    # mpc_command_interval, leaving ~40ms of margin. The producer
                    # then barely made one plan per send window (avg step
                    # 212-219ms), missed command ticks, and convergence got worse,
                    # not better. Raise mpc_command_interval first if this is ever
                    # increased again.
                    'mpc_warm_start_iters': 25,
                    'mpc_cold_start_iters': 100,
                    'mpc_mppi_config_file': LaunchConfiguration('mpc_mppi_config_file'),
                    'mpc_lbfgs_config_file': LaunchConfiguration('mpc_lbfgs_config_file'),
                    # Both args were DECLARED above but never passed to the node,
                    # so mpc_debug stayed at its in-code default (False) and a run
                    # produced no CSV at all unless someone remembered to
                    # `ros2 param set` it by hand — which is exactly the trap the
                    # declaration comment above warns about. ParameterValue with an
                    # explicit value_type is required: a LaunchConfiguration is a
                    # string substitution, and 'false' as a string would not match
                    # the node's bool parameter type.
                    'lbfgs_debug': ParameterValue(
                        LaunchConfiguration('mpc_debug'), value_type=bool),
                    'diagnostic_csv_dir': ParameterValue(
                        LaunchConfiguration('diag_dir'), value_type=str),
                    'max_joint_accel_dps': ParameterValue(
                        LaunchConfiguration('max_joint_accel_dps'), value_type=float),
                    'max_attempts': 10,
                    # Sparse voxel topic publish rate (Hz); <= 0 disables it.
                    'sparse_voxel_publish_rate': 5.0,
                    # Demi-vie du poids d'un voxel qui n'est plus observé. Il sort de
                    # la carte de collision au bout d'environ 3x cette valeur (~2 s).
                    # Le facteur par appel integrate() en est déduit à partir du débit
                    # combiné des caméras (frame_rate_hz dans cameras.yaml), donc cette
                    # valeur reste valable si on ajoute une caméra ou change un fps.
                    'decay_half_life_s': 0.7,
                    'use_sim_time': _use_sim_time,
                }],
            ),
            Node(
                package='curobo_ros',
                executable='robot_segmentation',
                output='screen',
                respawn=True,
                respawn_delay=2.0,
                parameters=[{
                    'robot_base_frame': 'dsr01/world',
                    'robot_config_file': leeloo_curobo_config,
                    # One node segments both depth cameras (shared robot
                    # kinematics + shared set_mask/remove_mask state, so a
                    # single SetMask call masks a grasped object out of both
                    # views). See cameras.yaml: oak_wrist consumes
                    # masked_depth_image_oak_wrist instead of the raw OAK topic.
                    'camera_names': ['kinect', 'oak_wrist'],
                    'kinect.depth_image_topic': '/depth_to_rgb/image_raw',
                    'kinect.camera_info_topic': '/depth_to_rgb/camera_info',
                    'kinect.output_topic': 'masked_depth_image',
                    'oak_wrist.depth_image_topic': '/oak/stereo/image_raw',
                    'oak_wrist.camera_info_topic': '/oak/stereo/camera_info',
                    'oak_wrist.output_topic': 'masked_depth_image_oak_wrist',
                    'use_sim_time': _use_sim_time,
                }],
            ),

            Node(
                package='leeloo',
                executable='execute_trajectory',
                output='screen',
                respawn=True,
                respawn_delay=2.0,
                parameters=[{
                    'command_period': 0.03,
                    'speedj_time': 0.02,
                    'use_sim_time': _use_sim_time,
                }],
            ),
            Node(
                package='leeloo',
                executable='ridgeback_control',
                output='screen',
                respawn=True,
                respawn_delay=2.0,
                parameters=[{'use_sim_time': _use_sim_time}],
            ),
        ]),
    ])
