from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory


def generate_launch_description():
    kinect_driver_share = get_package_share_directory('azure_kinect_ros_driver')
    leeloo_share        = get_package_share_directory('leeloo')

    kinect_launch_file = os.path.join(kinect_driver_share, 'launch', 'driver.launch.py')

    # Le driver OAK est résolu de façon TOLÉRANTE : sa résolution était
    # inconditionnelle et faisait échouer tout le bringup avec PackageNotFoundError
    # quand depthai_ros_driver_v3 n'est pas installé — y compris en sim:=true, où
    # ce driver n'est jamais lancé (les images viennent des capteurs Gazebo).
    # Absent → le bloc OAK n'est simplement pas ajouté ; en réel avec 'oak' dans
    # enabled_cameras, l'avertissement ci-dessous dit quoi installer.
    try:
        oak_driver_share = get_package_share_directory('depthai_ros_driver_v3')
        oak_launch_file  = os.path.join(oak_driver_share, 'launch', 'sr_rgbd_pcl.launch.py')
        _oak_available   = True
    except PackageNotFoundError:
        _oak_available = False
        # Chemin factice : PythonLaunchDescriptionSource ne touche au fichier qu'à
        # l'évaluation, et la condition _oak_enabled ci-dessous est forcée à False
        # dans ce cas — il n'est donc jamais lu.
        oak_launch_file = os.path.join(leeloo_share, 'launch', '_oak_driver_absent.launch.py')
        print("[cameras.launch] depthai_ros_driver_v3 introuvable — la caméra OAK-D "
              "ne sera pas lancée (sans effet en sim:=true, où leeloo_gazebo publie "
              "/oak/stereo/* depuis Gazebo).")

    # Liste de caméras à activer, séparées par des virgules (ex: 'kinect',
    # 'oak', 'kinect,oak'). Une liste plutôt qu'une énumération figée à 2 noms :
    # une 3e caméra n'a qu'à ajouter son propre nom + sa propre condition
    # ci-dessous, sans toucher au mécanisme de sélection.
    #
    # `and not sim` : en simulation les images de profondeur viennent des
    # capteurs Gazebo de leeloo_gazebo, sur EXACTEMENT les mêmes topics
    # (/depth_to_rgb/*, /oak/stereo/*). Lancer les drivers en plus ferait deux
    # publishers concurrents sur ces topics — et ils échoueraient de toute façon,
    # faute de matériel USB.
    _kinect_enabled = IfCondition(PythonExpression(
        ["'kinect' in '", LaunchConfiguration('enabled_cameras'), "'.split(',')",
         " and not '", LaunchConfiguration('sim'), "'.lower() in ('true', '1')"]))
    _oak_enabled = IfCondition(PythonExpression(
        ["'oak' in '", LaunchConfiguration('enabled_cameras'), "'.split(',')",
         " and not '", LaunchConfiguration('sim'), "'.lower() in ('true', '1')",
         f" and {_oak_available}"]))

    return LaunchDescription([
        DeclareLaunchArgument(
            'enabled_cameras', default_value='kinect,oak',
            description="Caméras à lancer, séparées par des virgules (ex: 'kinect', 'oak', 'kinect,oak')."),

        # En simulation, aucun driver caméra n'est lancé : leeloo_gazebo publie
        # sur les mêmes topics depuis des capteurs Gazebo.
        DeclareLaunchArgument('sim', default_value='false'),

        # ── k4a GPU depth engine ───────────────────────────────────────────────
        # libdepthengine.so uses OpenGL compute shaders for depth processing.
        # GLVND defaults to Mesa (llvmpipe, CPU) via Xvfb's vendor advertisement.
        # Forcing the NVIDIA vendor makes the depth engine run on the Tegra GPU.
        SetEnvironmentVariable('DISPLAY', ':99'),
        SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia'),

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

        # OAK-D SR camera arguments — eye-in-hand sur le poignet Doosan
        # (dsr01/link_6). cam_pos_*/cam_roll/pitch/yaw = offset physique du
        # montage sur le poignet, à mesurer/calibrer sur le robot réel.
        DeclareLaunchArgument('oak_camera_model', default_value='OAK-D-SR'),
        DeclareLaunchArgument('oak_parent_frame', default_value='dsr01/link_6'),
        # Pose de test pour dérisquer le pipeline (pas une calibration réelle) —
        # voir /root/.claude/plans/comment-je-peux-ajouter-serene-otter.md
        DeclareLaunchArgument('oak_cam_pos_x',    default_value='0.0924476130'),
        DeclareLaunchArgument('oak_cam_pos_y',    default_value='-0.01320633498'),
        DeclareLaunchArgument('oak_cam_pos_z',    default_value='0.024658523349'),
        DeclareLaunchArgument('oak_cam_roll',     default_value='3.114168001'),
        DeclareLaunchArgument('oak_cam_pitch',    default_value='-1.323787043'),
        DeclareLaunchArgument('oak_cam_yaw',      default_value='-3.113287621'),
        # Filtres stéréo on-device (fps, seuils de portée, speckle, conf) —
        # voir leeloo/config/oak_sr_depth.yaml.
        DeclareLaunchArgument(
            'oak_params_file',
            default_value=os.path.join(leeloo_share, 'config', 'oak_sr_depth.yaml')),

        # ── Azure Kinect (scope global : garde ses topics de données globaux) ──
        # driver.launch.py gère son propre robot_state_publisher (frames caméra).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(kinect_launch_file),
            condition=_kinect_enabled,
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

        # ── Caméra OAK-D SR (scope global, eye-in-hand sur dsr01/link_6) ────────
        # sr_rgbd_pcl.launch.py gère son propre robot_state_publisher (modèle 3D +
        # frames caméra), rattaché à dsr01/link_6 via parent_frame + offset cam_*.
        # Vient en complément du Kinect (pas de remplacement) ; sa TF est ramenée
        # vers /leeloo/tf par le relais générique relay_global_tf (tf.launch.py).
        #
        # LD_PRELOAD d'un libusb sans backend udev : OBLIGATOIRE, sinon le driver
        # boucle sur "Failed to find device after booting: X_LINK_DEVICE_NOT_FOUND".
        # Le libusb système (compilé avec udev) ne voit pas l'OAK-D se ré-énumérer
        # après son boot, faute de démon udev dans le conteneur — cf. le bloc
        # libusb-noudev dans aarch64_jazzy.dockerfile. Le GroupAction limite la
        # variable au driver OAK : le reste du graphe (Azure Kinect notamment)
        # garde le libusb système.
        GroupAction(
            condition=_oak_enabled,
            actions=[
                SetEnvironmentVariable(
                    'LD_PRELOAD', '/opt/libusb-noudev/lib/libusb-1.0.so.0'),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(oak_launch_file),
                    launch_arguments={
                        # Sans ça, 'name' hérite de la valeur déjà déclarée par
                        # dsr_bringup2_rviz.launch.py ('dsr01', inclus dans
                        # control.launch.py) : DeclareLaunchArgument ne réécrase
                        # pas un argument déjà déclaré. Le driver OAK se
                        # retrouvait alors nommé 'dsr01' et fusionné dans
                        # 'dsr01_container' — ses params (sr_rgbd.yaml, namespace
                        # /oak:) ne s'appliquaient plus (mismatch de nom), le driver
                        # retombait sur son pipeline par défaut (RGBD) et plantait
                        # sur ce capteur sans caméra couleur, en emportant tout le
                        # conteneur du bras avec lui.
                        'name': 'oak',
                        'camera_model': LaunchConfiguration('oak_camera_model'),
                        'parent_frame': LaunchConfiguration('oak_parent_frame'),
                        'cam_pos_x':    LaunchConfiguration('oak_cam_pos_x'),
                        'cam_pos_y':    LaunchConfiguration('oak_cam_pos_y'),
                        'cam_pos_z':    LaunchConfiguration('oak_cam_pos_z'),
                        'cam_roll':     LaunchConfiguration('oak_cam_roll'),
                        'cam_pitch':    LaunchConfiguration('oak_cam_pitch'),
                        'cam_yaw':      LaunchConfiguration('oak_cam_yaw'),
                        'params_file':  LaunchConfiguration('oak_params_file'),
                    }.items()
                ),
            ],
        ),
    ])
