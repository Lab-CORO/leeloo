from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetRemap
import os

from ament_index_python.packages import get_package_share_directory


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
    leeloo_share = get_package_share_directory('leeloo')

    # Même liste que cameras.launch.py : la TF de la Kinect n'a de sens que si
    # son driver tourne. `and not sim` : en simulation la pose de la Kinect est
    # NOMINALE (lien fixe sur camera_mount dans la description de leeloo_gazebo,
    # publié par son RSP), il n'y a pas de résultat de calibration à rejouer.
    _kinect_enabled = IfCondition(PythonExpression(
        ["'kinect' in '", LaunchConfiguration('enabled_cameras'), "'.split(',')",
         " and not '", LaunchConfiguration('sim'), "'.lower() in ('true', '1')"]))

    # leeloo_viz_state_publisher : remplacé en sim par le RSP de leeloo_gazebo,
    # qui publie une description SURENSEMBLE de leeloo.urdf (mêmes frames, plus
    # les liens caméra et les blocs sim). Deux RSP sur le même arbre
    # produiraient des transformes statiques dupliquées et deux publishers
    # concurrents sur /leeloo/robot_description.
    _not_sim = IfCondition(PythonExpression(
        ["not '", LaunchConfiguration('sim'), "'.lower() in ('true', '1')"]))

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

    return LaunchDescription([
        DeclareLaunchArgument('tf_remap',        default_value='/leeloo/tf'),
        DeclareLaunchArgument('tf_static_remap', default_value='/leeloo/tf_static'),
        DeclareLaunchArgument(
            'enabled_cameras', default_value='kinect,oak',
            description="Caméras à lancer, séparées par des virgules (ex: 'kinect', 'oak', 'kinect,oak')."),
        DeclareLaunchArgument('sim', default_value='false'),

        # ── Relais TF : ramènent les arbres externes/globaux vers /leeloo ──────
        # Ridgeback (autre machine, namespace /r100_0597).
        _tf_relay('relay_ridgeback_tf',        '/r100_0597/tf',        '/leeloo/tf'),
        _tf_relay('relay_ridgeback_tf_static', '/r100_0597/tf_static', '/leeloo/tf_static'),
        # Bras Doosan + driver Kinect (publient sur le /tf[_static] global).
        _tf_relay('relay_global_tf',           '/tf',                  '/leeloo/tf'),
        _tf_relay('relay_global_tf_static',    '/tf_static',           '/leeloo/tf_static'),

        GroupAction([
            SetRemap('/tf',        LaunchConfiguration('tf_remap')),
            SetRemap('/tf_static', LaunchConfiguration('tf_static_remap')),

            # RSP structure : publie default_mount→structure_base→…→robot_mount→dsr01/world
            # dans /leeloo/tf_static (le maillon manquant entre Ridgeback et bras).
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='structure_state_publisher',
                output='screen',
                parameters=[{'robot_description': structure_description}],
            ),

            # Pose de la Kinect : relit le résultat de calibration main-œil
            # sauvegardé et publie base_link→camera_base dans /leeloo/tf.
            # Ce n'est PAS un node de calibration (il ne fait que rejouer un
            # résultat existant) : il doit tourner en permanence, y compris
            # quand la chaîne de calibration est désactivée
            # (enable_calibration:=false dans calibration.launch.py).
            Node(
                package='leeloo_calibration',
                executable='kinect_tf_computation_node',
                name='kinect_tf_computation_node',
                output='screen',
                respawn=True,
                respawn_delay=2.0,
                condition=_kinect_enabled,
                parameters=[{
                    'calibration_result_file': '/home/ros2_ws/src/leeloo_calibration/config/kinect_hand_eye_result.yaml',
                    'robot_base_frame': 'dsr01/world',
                }],
            ),

            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='leeloo_viz_state_publisher',
                output='screen',
                condition=_not_sim,
                parameters=[{'robot_description': leeloo_combined_description}],
                remappings=[
                    ('robot_description', '/leeloo/robot_description'),
                ],
            ),
        ]),
    ])
