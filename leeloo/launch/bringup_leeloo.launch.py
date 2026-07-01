from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
    GroupAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
import os
import yaml

from ament_index_python.packages import get_package_share_directory


# ─────────────────────────────────────────────────────────────────────────────
# Arbre TF unifié /leeloo
# ─────────────────────────────────────────────────────────────────────────────
# Tout le robot Leeloo (Ridgeback + structure + bras Doosan + Kinect) est
# rassemblé dans UN seul arbre TF publié sur /leeloo/tf et /leeloo/tf_static.
#
# Sources (chacune propriétaire de sa portion → pas de double publication) :
#   • Ridgeback (autre machine, /r100_0597/tf[_static]) ──relais──▶ /leeloo
#   • Doosan    (bras, /tf[_static] global)             ──relais──▶ /leeloo
#   • Kinect    (driver, /tf_static global)             ──relais──▶ /leeloo
#   • Structure (mid_mount→…→base_0)   ─ RSP statique, écrit direct ─▶ /leeloo
#   • Caméra    (base_link→camera_base) ─ node calibration, direct ──▶ /leeloo
#
# Les nodes "côté Leeloo" (RSP structure, calibration, curobo, segmentation,
# markertracker, handeye) sont enveloppés dans un GroupAction + SetRemap qui
# redirige UNIQUEMENT /tf et /tf_static vers /leeloo/tf[_static]. Les topics de
# DONNÉES (images, services dsr01, etc.) restent globaux et inchangés.
#
# Le bras Doosan et le driver Kinect restent lancés au scope global (on ne casse
# pas leurs internes) et leurs TF sont ramenés par relais (topic_tools/relay).
# ─────────────────────────────────────────────────────────────────────────────


def _tf_relay(name, src, dst):
    """topic_tools/relay : recopie le topic TF `src` vers `dst`."""
    return Node(
        package='topic_tools',
        executable='relay',
        name=name,
        arguments=[src, dst],
        output='screen',
        respawn=True,
        respawn_delay=2.0,
    )


def generate_launch_description():
    # Package share paths
    dsr_bringup_share   = get_package_share_directory('dsr_bringup2')
    kinect_driver_share = get_package_share_directory('azure_kinect_ros_driver')
    handeye_share       = get_package_share_directory('hand_eye_calibration')
    leeloo_share        = get_package_share_directory('leeloo')

    # Launch file paths
    dsr_launch_file    = os.path.join(dsr_bringup_share,   'launch', 'dsr_bringup2_rviz.launch.py')
    kinect_launch_file = os.path.join(kinect_driver_share, 'launch', 'driver.launch.py')
    handeye_launch_file = os.path.join(handeye_share,      'calibration.launch.py')

    # CuRobo config — read base_link and urdf_path from the YAML so they stay
    # consistent with what the planner reads at runtime (single source of truth).
    # leeloo_curobo_config = os.path.join(leeloo_share, 'config', 'leeloo_curobo.yaml')
    # with open(leeloo_curobo_config, 'r') as f:
    #     _cfg = yaml.safe_load(f)
    # _kin = _cfg.get('robot_cfg', {}).get('kinematics', {})
    # curobo_base_link = _kin.get('base_link', 'base_link')
    # curobo_urdf_path = _kin.get('urdf_path', os.path.join(leeloo_share, 'urdf', 'leeloo.urdf'))

    # URDF de la structure (pont default_mount → … → robot_mount → dsr01/world).
    # Tous joints fixes → robot_state_publisher publie dans /leeloo/tf_static.
    structure_urdf = os.path.join(leeloo_share, 'urdf', 'leeloo_structure.urdf')
    with open(structure_urdf, 'r') as f:
        structure_description = f.read()

    # URDF combiné (Ridgeback + structure + bras m1013) pour RViz uniquement.
    # Noms de frames alignés sur l'arbre TF réel /leeloo/tf[_static] (default_mount,
    # dsr01/world, dsr01/link_1…6) : RobotModel de RViz place chaque lien via un
    # lookup TF par nom, pas par FK propre — donc ce fichier n'a besoin QUE des bons
    # noms, pas de valeurs de joint exactes. Caméra volontairement absente (sa pose
    # est dynamique, cf. kinect_tf_computation_node) : elle ne sera pas rendue en
    # mesh mais son repère existe déjà dans le TF live.
    leeloo_combined_urdf = os.path.join(leeloo_share, 'urdf', 'leeloo.urdf')
    with open(leeloo_combined_urdf, 'r') as f:
        leeloo_combined_description = f.read()

    # ── Groupe "côté Leeloo" : TF redirigés vers /leeloo, données inchangées ──
    leeloo_tf_group = GroupAction([
        SetRemap('/tf',        '/leeloo/tf'),
        SetRemap('/tf_static', '/leeloo/tf_static'),

        # RSP structure : publie default_mount→structure_base→…→robot_mount→dsr01/world
        # dans /leeloo/tf_static (le maillon manquant entre Ridgeback et bras).
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='structure_state_publisher',
            output='screen',
            parameters=[{'robot_description': structure_description}],
        ),

        # RSP dédié RViz : publie l'URDF combiné (Ridgeback+structure+bras) sur
        # /leeloo/robot_description pour que RViz affiche les meshes du robot
        # entier. Son /tf_static suit le remap ambiant du groupe comme tout le
        # monde (→ /leeloo/tf_static) : il republie les mêmes fixed joints que
        # structure_state_publisher/relais (valeurs identiques, doublon inoffensif).
        # Pas de joint_states reçu (rien ne publie sur /joint_states global) →
        # aucune TF articulée produite ici, pas de conflit avec les valeurs
        # dynamiques réelles (bras, roues) qui viennent des vraies sources.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='leeloo_viz_state_publisher',
            output='screen',
            parameters=[{'robot_description': leeloo_combined_description}],
            remappings=[
                ('robot_description', '/leeloo/robot_description'),
            ],
        ),

        # Caméra : dsr01/world→camera_base depuis le résultat hand-eye.
        # Ancrée sur la racine du bras Doosan (frame_prefix dsr01/) → caméra et
        # bras partagent dsr01/world, indépendamment du Ridgeback.
        Node(
            package='leeloo_calibration',
            executable='kinect_tf_computation_node',
            name='kinect_tf_computation_node',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[{
                'calibration_result_file': '/home/ros2_ws/src/leeloo_calibration/config/kinect_hand_eye_result.yaml',
                'robot_base_frame': 'dsr01/world',
            }],
        ),

        # CuRobo trajectory planner — launched directly to avoid the extra nodes
        # that gen_traj.launch.py pulls in (standalone RSP, emulator JSP, preview
        # nodes). Those are irrelevant on the real robot and conflict with the
        # unified /leeloo TF tree (duplicate robot_state_publisher on /tf).
        Node(
            package='curobo_ros',
            executable='curobo_trajectory_planner',
            name='curobo_trajectory_planner',
            output='screen',
            respawn=True,
            respawn_delay=5.0,
            parameters=[{
                # 'robot_config_file': leeloo_curobo_config,
                'cameras_config_file': LaunchConfiguration('cameras_config_file'),
                'base_link': "dsr01/world",
                'world_file': LaunchConfiguration('world_file'),
                'voxel_size': LaunchConfiguration('voxel_size'),
                # Mapper ESDF grid: 128^3 @ 0.02 m centred on robot base.
                'mapper_extent_xyz': [2.56, 2.56, 2.56],
                'mapper_grid_center': [0.0, 0.0, 0.0],
                # Depth frame resolution fed to mapper.integrate().
                # Must match the Azure Kinect depth_to_rgb output (1280x720).
                'mapper_image_width': 1280,
                'mapper_image_height': 720,
                # Sparse voxel topic publish rate (Hz); <= 0 disables it.
                'sparse_voxel_publish_rate': 7.0,
                # GPU rasterization of analytic primitives is disabled at startup
                # to avoid NVRTC kernel compilation (~90 s warmup).
                # Enable at runtime: ros2 param set /curobo_trajectory_planner
                #                    enable_primitives_rasterization true
                'enable_primitives_rasterization': True,
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
            }],
        ),

        Node(
            package='leeloo',
            executable='execute_trajectory',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
        ),

        # ── Calibration (délai : attendre que la TF caméra soit dispo) ──
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
                    # Préfixe des TF par marqueur : un TF marker_<id> par ID détecté
                    # (ex: marker_21 = marqueur de calibration, cf. tracking_marker_frame
                    # dans le hand_eye_calibration ci-dessous).
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
                    # Frames du bras Doosan préfixées par frame_prefix='dsr01/'.
                    # On ancre la calibration sur la racine du bras (dsr01/world,
                    # confondue avec base_link) et l'effecteur dsr01/link_6, pour
                    # que la relation caméra↔bras soit indépendante de la base mobile.
                    'robot_base_frame':        'dsr01/world',
                    'robot_effector_frame':    'dsr01/link_6',
                    'tracking_base_frame':     'rgb_camera_link',
                    # ID 21 = marqueur dédié à la calibration (le markertracker publie
                    # un TF par ID détecté : marker_<id>, cf. marker_frame_id ci-dessous).
                    'tracking_marker_frame':   'marker_21',
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
    ])

    return LaunchDescription([
        # ── Fix "sequence size exceeds remaining buffer" ───────────────────────
        # FastDDS floods its discovery buffer when N nodes start simultaneously.
        # RMW_FASTRTPS_USE_QOS_FROM_XML=0 forces FastDDS to use its built-in QoS
        # defaults instead of any XML profile. FASTRTPS_DEFAULT_PROFILES_FILE must
        # NOT be set to empty string — that triggers a realpath("") XMLPARSER error.
        SetEnvironmentVariable('RMW_FASTRTPS_USE_QOS_FROM_XML', '0'),

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

        DeclareLaunchArgument('cameras_config_file', default_value='/home/ros2_ws/src/leeloo/config/cameras.yaml'),
        DeclareLaunchArgument('world_file',          default_value='/home/ros2_ws/src/leeloo/config/leeloo_world.yaml'),
        DeclareLaunchArgument('voxel_size',          default_value='0.02'),

        # Kinect camera arguments — tunable without editing this file.
        # NFOV_UNBINNED uses ~40% less USB bandwidth than WFOV_UNBINNED,
        # which prevents the "Failed to poll cameras" FATAL crash.
        DeclareLaunchArgument('depth_mode',        default_value='NFOV_UNBINNED'),
        DeclareLaunchArgument('color_enabled',     default_value='true'),
        DeclareLaunchArgument('color_format',      default_value='bgra'),
        DeclareLaunchArgument('color_resolution',  default_value='720P'),
        DeclareLaunchArgument('fps',               default_value='5'),
        DeclareLaunchArgument('rgb_point_cloud',   default_value='false'),
        DeclareLaunchArgument('point_cloud',       default_value='false'),
        DeclareLaunchArgument('imu_rate_target',   default_value='100'),

        # ── Relais TF : ramènent les arbres externes/globaux vers /leeloo ──────
        # Ridgeback (autre machine, namespace /r100_0597).
        _tf_relay('relay_ridgeback_tf',        '/r100_0597/tf',        '/leeloo/tf'),
        _tf_relay('relay_ridgeback_tf_static', '/r100_0597/tf_static', '/leeloo/tf_static'),
        # Bras Doosan + driver Kinect (publient sur le /tf[_static] global).
        _tf_relay('relay_global_tf',           '/tf',                  '/leeloo/tf'),
        _tf_relay('relay_global_tf_static',    '/tf_static',           '/leeloo/tf_static'),

        # ── Doosan bringup (scope global : ne pas casser les internes dsr01) ───
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(dsr_launch_file),
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

        # ── Azure Kinect (scope global : garde ses topics de données globaux) ──
        # driver.launch.py gère son propre robot_state_publisher (frames caméra).
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

        # ── Nodes côté Leeloo (TF → /leeloo, données inchangées) ──────────────
        leeloo_tf_group,
    ])
