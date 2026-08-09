"""
leeloo_calibration/launch/calibration.launch.py
================================================
Section calibration main-œil de Leeloo (extraite de bringup_leeloo.launch.py) :
détection des marqueurs ArUco, un handeye_server par caméra, et l'orchestrateur
auto_calibration_server.

Applique lui-même son remap /tf -> /leeloo/tf (paramétrable via tf_remap /
tf_static_remap, propagés explicitement par bringup_leeloo.launch.py) : ce
fichier reste donc lançable seul (`ros2 launch leeloo_calibration
calibration.launch.py`) sans dépendre d'être inclus dans un GroupAction parent.

Ajouter une caméra = ajouter un bloc handeye_server ci-dessous (remappé avec
_handeye_remaps) ; rien à déclarer côté auto_calibration_server, qui construit
ses clients de service à la volée à partir du champ `handeye_server` du goal
RunCalibration (voir auto_calibration_server.py:_get_clients).

L'argument `enable_calibration` (défaut true) coupe toute la chaîne de
calibration main-œil : markertrackers dédiés au marqueur de calibration,
handeye_server et auto_calibration_server. Utile en exploitation, où aucun
marqueur de calibration n'est en vue : handeye_server ne crée ses services
qu'APRÈS avoir résolu `tracking_base_frame -> tracking_marker_frame` dans tf
(easy_handeye2/handeye_server.py:48-51), donc sans marqueur il retente
indéfiniment toutes les ~10 s en logguant des ERROR, et auto_calibration_server
finit par abandonner sur timeout. Le markertracker du marqueur applicatif, lui,
reste lancé : il ne sert pas à la calibration.

À NE PAS remettre ici : kinect_tf_computation_node ne calibre rien, il rejoue
le résultat de calibration sauvegardé pour publier la TF de la Kinect — il vit
donc dans leeloo/launch/tf.launch.py et tourne en permanence.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetRemap


# Suffixes relatifs des services publiés par easy_handeye2 sous
# CALIBRATION_NAMESPACE = '/easy_handeye2/calibration/' (easy_handeye2/__init__.py).
# Ce nom absolu est fixe dans le paquet, identique quel que soit le nom du
# handeye_server -> sans remap, deux handeye_server actifs simultanément
# entrent en collision de service (et de nom de nœud, géré séparément via `name=`).
_HANDEYE_TOPICS = [
    'list_algorithms', 'set_algorithm', 'get_current_transforms',
    'get_sample_list', 'take_sample', 'remove_sample',
    'save_samples', 'load_samples', 'compute_calibration', 'save_calibration',
    'check_starting_pose', 'enumerate_target_poses', 'select_target_pose',
    'plan_to_selected_target_pose', 'execute_plan',
]


def _handeye_remaps(prefix: str):
    """Remappe les services absolus d'easy_handeye2 vers /easy_handeye2_<prefix>/...
    pour que N handeye_server tournent simultanément sans collision de service."""
    return [
        (f'/easy_handeye2/calibration/{t}', f'/easy_handeye2_{prefix}/calibration/{t}')
        for t in _HANDEYE_TOPICS
    ]


def _camera_enabled(camera: str) -> list:
    """Fragment d'expression Python : la caméra est dans `enabled_cameras`."""
    return ["'", camera, "' in '", LaunchConfiguration('enabled_cameras'), "'.split(',')"]


def _calibration_enabled() -> list:
    """Fragment d'expression Python : `enable_calibration` vaut vrai."""
    return ["'", LaunchConfiguration('enable_calibration'),
            "'.lower() in ('true', '1', 'yes', 'on')"]


def generate_launch_description():
    # Même liste que cameras.launch.py (même argument, même mécanisme de
    # propagation CLI) : ne démarre les marqueurs/handeye_server d'une caméra
    # que si son driver est réellement lancé, sinon markertracker_node attend
    # indéfiniment un camera_info qui n'arrivera jamais et plante (cf. bug
    # remonté : 'Failed to get camera info after multiple attempts' laisse le
    # nœud à moitié construit, puis AttributeError sur new_msg_available).
    _kinect_enabled = IfCondition(PythonExpression(_camera_enabled('kinect')))
    _oak_enabled = IfCondition(PythonExpression(_camera_enabled('oak')))

    # Idem, ET la calibration main-œil est demandée (cf. docstring du module).
    _calib_enabled = IfCondition(PythonExpression(_calibration_enabled()))
    _kinect_calib_enabled = IfCondition(PythonExpression(
        _camera_enabled('kinect') + [' and '] + _calibration_enabled()))
    _oak_calib_enabled = IfCondition(PythonExpression(
        _camera_enabled('oak') + [' and '] + _calibration_enabled()))

    return LaunchDescription([
        DeclareLaunchArgument('tf_remap',        default_value='/leeloo/tf'),
        DeclareLaunchArgument('tf_static_remap', default_value='/leeloo/tf_static'),
        DeclareLaunchArgument('enabled_cameras', default_value='kinect,oak'),
        DeclareLaunchArgument(
            'enable_calibration', default_value='true',
            description="Démarre la chaîne de calibration main-œil (markertrackers "
                        "du marqueur de calibration, handeye_server, "
                        "auto_calibration_server). Mettre à false en exploitation, "
                        "quand aucun marqueur de calibration n'est en vue."),

        GroupAction([
        SetRemap('/tf',        LaunchConfiguration('tf_remap')),
        SetRemap('/tf_static', LaunchConfiguration('tf_static_remap')),

        # ── Marker for azure kinect calibration (délai : attendre que la TF caméra soit dispo) ──
        TimerAction(period=5.0, condition=_kinect_calib_enabled, actions=[
            Node(
                package='ros2_markertracker',
                executable='markertracker_node',
                name='calibration_marker_detection',
                output='screen',
                namespace='/ros2_markertracker',
                respawn=True,
                respawn_delay=5.0,
                parameters=[{
                    'input_image_topic':          '/rgb/image_raw',
                    'publish_topic_image_result': True,
                    'camera_info_topic':          '/rgb/camera_info',
                    'camera_frame_id':            'rgb_camera_link',

                    # ── Mode ChArUco board ──
                    'use_charuco':                False,
                    'aruco_dictionary_id':        'DICT_APRILTAG_36H11',
                    'marker_length':              10.0,
                    'marker_frame_id':            'marker',
                    'ignore_marker_ids_array':    18,


                }]
            ),
        ]),
        # Marker for the appication / task (regarde /rgb/image_raw -> dépend aussi du Kinect)
        TimerAction(period=5.0, condition=_kinect_enabled, actions=[
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
                    'camera_frame_id':            'rgb_camera_link',

                    # ── Mode ChArUco board ──
                    'use_charuco':                False,
                    'aruco_dictionary_id':        'DICT_APRILTAG_36H11',
                    'marker_length':              8.6,
                    'marker_frame_id':            'marker',
                    'ignore_marker_ids_array':    17,
                }]
            ),
        ]),
        # Marker for oak-D calibration
        TimerAction(period=5.0, condition=_oak_calib_enabled, actions=[
            Node(
                package='ros2_markertracker',
                executable='markertracker_node',
                name='oak_marker_detection',
                output='screen',
                namespace='/ros2_markertracker',
                respawn=True,
                respawn_delay=5.0,
                parameters=[{
                    'input_image_topic':          '/oak/right/image_raw',
                    'publish_topic_image_result': True,
                    'camera_info_topic':          '/oak/right/camera_info',
                    'camera_frame_id':            'oak_right_camera_optical_frame',
                    'use_charuco':                False,
                    'aruco_dictionary_id':        'DICT_APRILTAG_36H11',
                    'marker_length':              10.0,
                    'marker_frame_id':            'marker',
                    'ignore_marker_ids_array':    17,
                }]
            ),
        ]),

        TimerAction(period=5.0, condition=_calib_enabled, actions=[
            # Calibration Eye on Base (Kinect)
            Node(
                package='easy_handeye2',
                executable='handeye_server',
                name='kinect_handeye_server',
                output='screen',
                condition=_kinect_enabled,
                remappings=_handeye_remaps('kinect'),
                parameters=[{
                    'name':                    'kinect_calib',
                    'calibration_type':        'eye_on_base',
                    # Frames du bras Doosan préfixées par frame_prefix='dsr01/'.
                    # On ancre la calibration sur la racine du bras (dsr01/world,
                    # confondue avec base_link) et l'effecteur dsr01/link_6, pour
                    # que la relation caméra↔bras soit indépendante de la base mobile.
                    'robot_base_frame':        'dsr01/world',
                    'robot_effector_frame':    'dsr01/link_6',
                    'tracking_base_frame':     'rgb_camera_link',
                    # ID 18 = marqueur dédié à la calibration.
                    'tracking_marker_frame':   'marker_18',
                }],
            ),

            # Calibration Eye in Hand (OAK-D)
            Node(
                package='easy_handeye2',
                executable='handeye_server',
                name='oak_handeye_server',
                output='screen',
                condition=_oak_enabled,
                remappings=_handeye_remaps('oak'),
                parameters=[{
                    'name':                    'oak_d_SR_calib',
                    'calibration_type':        'eye_in_hand',
                    # Frames du bras Doosan préfixées par frame_prefix='dsr01/'.
                    # On ancre la calibration sur la racine du bras (dsr01/world,
                    # confondue avec base_link) et l'effecteur dsr01/link_6, pour
                    # que la relation caméra↔bras soit indépendante de la base mobile.
                    'robot_base_frame':        'dsr01/world',
                    'robot_effector_frame':    'dsr01/link_6',
                    # Racine de l'arbre TF OAK-D (argument parent_frame du driver
                    # depthai), PAS oak_right_camera_optical_frame : ce frame a déjà
                    # un parent publié en interne par le robot_state_publisher du
                    # driver — publier un second parent depuis dsr01/link_6 via
                    # easy_handeye2 entrerait en conflit. tf2 résout de façon
                    # transitive oak → … → oak_right_camera_optical_frame → marker_18
                    # (lookup_transform standard, pas besoin d'adjacence), donc le
                    # calcul AX=XB reste valide contre la racine. Nom à confirmer
                    # via `ros2 run tf2_tools view_frames` (hypothèse 'oak').
                    'tracking_base_frame':     'oak',
                    # ID 18 = marqueur dédié à la calibration.
                    'tracking_marker_frame':   'marker_18',
                }],
            ),

            Node(
                package='leeloo_calibration',
                executable='auto_calibration_server',
                name='auto_calibration_server',
                output='screen',
                respawn=True,
                respawn_delay=5.0,
                parameters=[{
                    'enable_debug_capture': False,
                    'default_handeye_server': 'kinect',
                    'calibration_poses_file': '/home/ros2_ws/src/leeloo_calibration/config/calibration_poses.yaml',
                    'calibration_result_file': '/home/ros2_ws/src/leeloo_calibration/config/kinect_hand_eye_result.yaml',
                    'reload_calibration_service': '/kinect_tf_computation_node/reload_calibration',
                    'server_wait_timeout_s': 30.0,
                }],
            ),
        ]),
        ]),
    ])
