from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os

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
# markertracker, handeye) redirigent /tf et /tf_static vers /leeloo/tf[_static],
# chacun via son propre GroupAction + SetRemap paramétré par tf_remap /
# tf_static_remap (voir tf.launch.py, control.launch.py, leeloo_calibration/
# launch/calibration.launch.py). Les topics de DONNÉES (images, services dsr01,
# etc.) restent globaux et inchangés.
#
# Ce fichier n'est plus qu'un orchestrateur : il déclare les env vars globales
# et les arguments de remap TF (source de vérité unique), puis inclut les
# fichiers thématiques :
#   • cameras.launch.py      : Kinect + OAK-D SR
#   • tf.launch.py            : relais TF + robot_state_publisher (structure/viz)
#   • control.launch.py       : bras Doosan, ros2_control/gripper, curobo, execute_trajectory
#   • leeloo_calibration/launch/calibration.launch.py : markertracker, handeye_server, auto_calibration_server
# ─────────────────────────────────────────────────────────────────────────────


def generate_launch_description():
    leeloo_share      = get_package_share_directory('leeloo')
    calibration_share = get_package_share_directory('leeloo_calibration')

    cameras_launch_file     = os.path.join(leeloo_share, 'launch', 'cameras.launch.py')
    tf_launch_file          = os.path.join(leeloo_share, 'launch', 'tf.launch.py')
    control_launch_file     = os.path.join(leeloo_share, 'launch', 'control.launch.py')
    calibration_launch_file = os.path.join(calibration_share, 'launch', 'calibration.launch.py')

    _tf_remap_args = {
        'tf_remap':        LaunchConfiguration('tf_remap'),
        'tf_static_remap': LaunchConfiguration('tf_static_remap'),
    }

    return LaunchDescription([
        # ── Fix "sequence size exceeds remaining buffer" ───────────────────────
        # FastDDS floods its discovery buffer when N nodes start simultaneously.
        # RMW_FASTRTPS_USE_QOS_FROM_XML=0 forces FastDDS to use its built-in QoS
        # defaults instead of any XML profile. FASTRTPS_DEFAULT_PROFILES_FILE must
        # NOT be set to empty string — that triggers a realpath("") XMLPARSER error.
        SetEnvironmentVariable('RMW_FASTRTPS_USE_QOS_FROM_XML', '0'),

        # Nom des topics remappés depuis /tf, /tf_static côté "Leeloo" — seule
        # source de vérité, propagée explicitement à chaque fichier inclus.
        DeclareLaunchArgument('tf_remap',        default_value='/leeloo/tf'),
        DeclareLaunchArgument('tf_static_remap', default_value='/leeloo/tf_static'),

        # Chaîne de calibration main-œil (markertrackers du marqueur de
        # calibration, handeye_server, auto_calibration_server). À passer à
        # false en exploitation : sans marqueur de calibration en vue,
        # handeye_server ne crée jamais ses services et spamme des ERROR
        # toutes les ~10 s (voir calibration.launch.py).
        DeclareLaunchArgument('enable_calibration', default_value='true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(cameras_launch_file)),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tf_launch_file),
            launch_arguments=_tf_remap_args.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(control_launch_file),
            launch_arguments=_tf_remap_args.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(calibration_launch_file),
            launch_arguments={
                **_tf_remap_args,
                'enable_calibration': LaunchConfiguration('enable_calibration'),
            }.items()),
    ])
