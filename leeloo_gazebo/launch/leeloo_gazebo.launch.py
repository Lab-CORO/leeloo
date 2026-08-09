"""Simulated Leeloo hardware in Gazebo Sim, built from leeloo's URDF.

This launch file is a HARDWARE SUBSTITUTE, not a second robot stack. Every topic
name and message type it exposes matches what the real drivers expose, so
curobo_ros, robot_segmentation, execute_trajectory, ridgeback_control and grasp
all run unmodified against it:

    arm      curobo -> /leeloo/execute_trajectory -> execute_trajectory
             -> /dsr01/dsr_controller2/speedj_rt_stream -> dsr_sim_bridge
             -> arm_velocity_controller (velocity) -> gz_ros2_control
             feedback: /joint_states -> dsr_sim_bridge -> /dsr01/joint_states
    base     /cmd_vel -> ridgeback_control -> /r100_0597/joy_teleop/cmd_vel
             -> base_sim_bridge -> gz MecanumDrive
    gripper  /robotiq_gripper_controller/gripper_cmd (ParallelGripperCommand)
    cameras  gz depth_camera -> /depth_to_rgb/{image_raw,camera_info}
                             -> /oak/stereo/{image_raw,camera_info}

Use `ros2 launch leeloo bringup_leeloo.launch.py sim:=true` for the full stack;
this file on its own gives simulated hardware plus (with standalone:=true, the
default) leeloo's ridgeback_control so the base responds to /cmd_vel.

leeloo is NOT modified by this file. leeloo.urdf ships a <ros2_control> block for
the real Robotiq driver (robotiq_driver/RobotiqGripperHardwareInterface, COM_port
/dev/ttyUSB0) which cannot run in simulation, and it has no camera links at all
(the Kinect's pose comes from hand-eye calibration at runtime, the OAK-D has its
own robot_state_publisher inside the depthai driver). Since leeloo.urdf is a
flat, already-xacro-expanded file with no macro to parameterize, this launch file
does XML surgery on it in memory: strip the real <ros2_control> block, append
sim <ros2_control> blocks, append the <gazebo> plugin tags, and inject the two
camera links + depth sensors. The files under leeloo/urdf are never touched.

Base: the real Ridgeback is holonomic (mecanum wheels) -- ridgeback_control
forwards linear.y as-is to the vendor driver, which mixes wheels in firmware.
leeloo.urdf's wheels are plain cylinder collisions with a single revolute axis
(no mecanum roller geometry), so wheel-ground contact in Gazebo is isotropic and
can only ever produce longitudinal force. That rules out a ros2_control
mecanum_drive_controller, which would compute correct wheel speeds that the
contact model then cannot turn into lateral motion.

An earlier version of this file claimed gz::sim::systems::MecanumDrive escaped
that limitation by being "kinematic". It does not: it converts the body twist
into the same four wheel joint velocities and relies on the same contact to move
the body, so it fails in exactly the same way. Base motion therefore uses
gz::sim::systems::VelocityControl, which applies the twist to the model itself --
the abstraction the real firmware provides -- paired with
gz::sim::systems::OdometryPublisher so odometry comes from the model's true pose
instead of a command-side model. See the base drive section below for the
measurements and for what this choice costs. Those plugins are declared in the
robot_description, so the base sits outside gz_ros2_control; only the arm and
gripper go through ros2_control.

Gripper <mimic> joints: contrary to what an earlier version of this file claimed,
they are NOT enforced by the physics engine in this configuration. Verified on
this install (gz-sim 8.11 / gz-physics 7.6 / sdformat 14.9): the URDF->SDF
conversion does emit <axis><mimic> correctly, and gz-sim's physics system does
request gz::physics::SetMimicConstraintFeature -- but only the
bullet-featherstone engine implements it. gz-sim's DEFAULT engine is
gz-physics-dartsim-plugin, which has no mimic support at all, so the constraint
is silently dropped and only the commanded knuckle moves. Switching the world to
bullet-featherstone would fix it, at the cost of changing the dynamics of the
whole robot (and of gz_ros2_control and MecanumDrive with it) for one gripper
feature -- not worth it. ros2_control (Jazzy) enforces mimic joints itself: it
reads the <mimic> tags from the URDF and gz_ros2_control honours them, for joints
that also appear in the <ros2_control> block. Hence all six gripper joints are
listed below, with a command interface on the leading one only. That solution is
engine-independent.
"""
import math
import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue

# Real gripper's closed-position convention -- leeloo.urdf's own (real-hardware)
# ros2_control block uses this value, so the simulated gripper starts in the
# same pose.
GRIPPER_CLOSED_POSITION = "0.7929"

ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

# Leading (commanded) gripper joint, then the five that follow it through the
# URDF's <mimic> tags. All six must be declared to ros2_control for it to apply
# the mimic relations -- see the module docstring.
GRIPPER_LEAD_JOINT = "robotiq_85_left_knuckle_joint"
GRIPPER_MIMIC_JOINTS = [
    "robotiq_85_right_knuckle_joint",
    "robotiq_85_left_inner_knuckle_joint",
    "robotiq_85_right_inner_knuckle_joint",
    "robotiq_85_left_finger_tip_joint",
    "robotiq_85_right_finger_tip_joint",
]

# Wheel layout, from leeloo.urdf (ridgeback.urdf.xacro): wheel_separation is 2x
# the 0.2755 m y-offset of front_left_wheel off its rocker; wheelbase is 2x the
# 0.319 m x-offset of front_rocker/rear_rocker off axle_link; wheel radius is the
# wheel collision cylinder's radius.
#
# Currently unreferenced: these fed MecanumDrive's wheel mixing, which was
# dropped in favour of VelocityControl (see the base drive section below).
# Kept because they are measured facts about the robot, and any future attempt
# to drive the wheel joints -- a mecanum controller on rollers, or anisotropic
# wheel friction -- needs exactly these three numbers.
WHEEL_SEPARATION = "0.5510"
WHEELBASE = "0.6380"
WHEEL_RADIUS = "0.0759"

# front_rocker is a passive suspension pivot (revolute, +/-0.087 rad) declared
# with effort="0" and no <dynamics> at all, so in Gazebo it oscillates freely.
# The real rocker is damped by its mounting; these values just stop the base
# from bouncing and are not a characterisation of the hardware.
ROCKER_DAMPING = "5.0"
ROCKER_FRICTION = "1.0"

# Standard REP-103 body -> optical rotation (X forward, Z up  ->  Z forward,
# X right, Y down). gz renders a camera along its link's +X, while depth data is
# interpreted in the optical frame, so every camera gets a body link carrying the
# sensor plus an optical child frame that the ROS messages reference.
OPTICAL_FROM_BODY_RPY = f"{-math.pi / 2} 0 {-math.pi / 2}"

# Camera image size. NOT freely tunable: curobo's mapper is configured with
# mapper_image_width/mapper_image_height 1280x720 in leeloo/launch/control.launch.py,
# and both cameras feed the same mapper.
CAMERA_WIDTH = "1280"
CAMERA_HEIGHT = "720"


def _add_camera(root, name, parent_link, xyz, rpy, optical_frame,
                gz_topic, horizontal_fov, near_clip, far_clip, update_rate):
    """Inject a depth camera: body link + optical frame + gz depth sensor.

    The body link carries a small inertia on purpose: a massless link joined by a
    fixed joint is a candidate for link lumping during URDF->SDF conversion,
    which would move the sensor onto the parent link and silently misplace it.
    """
    body_link = f"{name}_camera_body"

    link_el = ET.SubElement(root, "link", {"name": body_link})
    inertial_el = ET.SubElement(link_el, "inertial")
    ET.SubElement(inertial_el, "mass", {"value": "0.001"})
    ET.SubElement(inertial_el, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    ET.SubElement(inertial_el, "inertia", {
        "ixx": "1e-6", "ixy": "0", "ixz": "0",
        "iyy": "1e-6", "iyz": "0", "izz": "1e-6",
    })

    joint_el = ET.SubElement(root, "joint", {
        "name": f"{name}_camera_body_joint", "type": "fixed"})
    ET.SubElement(joint_el, "parent", {"link": parent_link})
    ET.SubElement(joint_el, "child", {"link": body_link})
    ET.SubElement(joint_el, "origin", {"xyz": xyz, "rpy": rpy})

    # The optical frame is what cameras.yaml names as the camera's frame_id, and
    # what curobo looks up in TF to place the depth points. It must exist as a
    # real link so robot_state_publisher emits it.
    optical_el = ET.SubElement(root, "link", {"name": optical_frame})
    optical_inertial = ET.SubElement(optical_el, "inertial")
    ET.SubElement(optical_inertial, "mass", {"value": "0.001"})
    ET.SubElement(optical_inertial, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    ET.SubElement(optical_inertial, "inertia", {
        "ixx": "1e-6", "ixy": "0", "ixz": "0",
        "iyy": "1e-6", "iyz": "0", "izz": "1e-6",
    })

    optical_joint = ET.SubElement(root, "joint", {
        "name": f"{name}_camera_optical_joint", "type": "fixed"})
    ET.SubElement(optical_joint, "parent", {"link": body_link})
    ET.SubElement(optical_joint, "child", {"link": optical_frame})
    ET.SubElement(optical_joint, "origin",
                  {"xyz": "0 0 0", "rpy": OPTICAL_FROM_BODY_RPY})

    gazebo_el = ET.SubElement(root, "gazebo", {"reference": body_link})
    sensor_el = ET.SubElement(gazebo_el, "sensor", {
        "name": f"{name}_depth", "type": "depth_camera"})
    ET.SubElement(sensor_el, "always_on").text = "1"
    ET.SubElement(sensor_el, "update_rate").text = str(update_rate)
    ET.SubElement(sensor_el, "visualize").text = "false"
    ET.SubElement(sensor_el, "topic").text = gz_topic
    # gz_frame_id is the tag that actually stamps the published frame_id: it is
    # read by gz-sensors (8.2), whereas optical_frame_id below exists only as an
    # sdformat schema element in this version and nothing at runtime consumes it.
    # Both are set to the optical link so the bridged Image/CameraInfo carry the
    # frame cameras.yaml expects. sdformat warns that gz_frame_id is "not defined
    # in SDF" and copies it through verbatim -- that warning is expected, and it
    # is precisely what makes gz-sensors see it.
    ET.SubElement(sensor_el, "gz_frame_id").text = optical_frame
    camera_el = ET.SubElement(sensor_el, "camera")
    ET.SubElement(camera_el, "horizontal_fov").text = str(horizontal_fov)
    image_el = ET.SubElement(camera_el, "image")
    ET.SubElement(image_el, "width").text = CAMERA_WIDTH
    ET.SubElement(image_el, "height").text = CAMERA_HEIGHT
    clip_el = ET.SubElement(camera_el, "clip")
    ET.SubElement(clip_el, "near").text = str(near_clip)
    ET.SubElement(clip_el, "far").text = str(far_clip)
    ET.SubElement(camera_el, "optical_frame_id").text = optical_frame


def _build_sim_robot_description(controllers_yaml_path, cameras, camera_poses):
    leeloo_share = get_package_share_directory("leeloo")
    urdf_path = os.path.join(leeloo_share, "urdf", "leeloo.urdf")

    tree = ET.parse(urdf_path)
    root = tree.getroot()

    # Drop the real hardware's <ros2_control> block(s) -- direct children of
    # <robot> -- so the sim ones below don't fight over the same joints.
    for ros2_control_el in root.findall("ros2_control"):
        root.remove(ros2_control_el)

    # Damp the passive suspension rocker (see ROCKER_DAMPING).
    for joint_el in root.findall("joint"):
        if joint_el.get("name") == "front_rocker":
            ET.SubElement(joint_el, "dynamics", {
                "damping": ROCKER_DAMPING, "friction": ROCKER_FRICTION})

    def add_joint(parent, name, command_interfaces, state_interfaces,
                  initial_value=None):
        joint_el = ET.SubElement(parent, "joint", {"name": name})
        for iface in command_interfaces:
            ET.SubElement(joint_el, "command_interface", {"name": iface})
        for iface in state_interfaces:
            state_el = ET.SubElement(joint_el, "state_interface", {"name": iface})
            if iface == "position" and initial_value is not None:
                ET.SubElement(state_el, "param",
                              {"name": "initial_value"}).text = initial_value

    def add_ros2_control_block(name, joints_spec):
        rc_el = ET.SubElement(root, "ros2_control",
                              {"name": name, "type": "system"})
        hw_el = ET.SubElement(rc_el, "hardware")
        ET.SubElement(hw_el, "plugin").text = "gz_ros2_control/GazeboSimSystem"
        for joint_name, cmd_ifaces, state_ifaces, initial in joints_spec:
            add_joint(rc_el, joint_name, cmd_ifaces, state_ifaces, initial)
        return rc_el

    # Arm exposes BOTH position and velocity command interfaces. velocity is the
    # production path (curobo/MPC -> speedj -> arm_velocity_controller); position
    # backs arm_controller, which is spawned inactive for standalone tests. Only
    # one of the two controllers may be active at a time, since they claim the
    # same joints.
    add_ros2_control_block(
        "leeloo_arm_sim",
        [(j, ["position", "velocity"], ["position", "velocity"], None)
         for j in ARM_JOINTS],
    )
    # All six gripper joints, command interface on the leading one only:
    # ros2_control refuses a command interface on an activated mimic joint, and
    # it needs the mimic joints present here to apply the URDF's <mimic> tags at
    # all (Gazebo will not -- see module docstring).
    add_ros2_control_block(
        "leeloo_gripper_sim",
        [(GRIPPER_LEAD_JOINT, ["position"], ["position", "velocity"],
          GRIPPER_CLOSED_POSITION)]
        + [(j, [], ["position", "velocity"], None) for j in GRIPPER_MIMIC_JOINTS],
    )

    gazebo_el = ET.SubElement(root, "gazebo")
    plugin_el = ET.SubElement(gazebo_el, "plugin", {
        "filename": "gz_ros2_control-system",
        "name": "gz_ros2_control::GazeboSimROS2ControlPlugin",
    })
    ET.SubElement(plugin_el, "parameters").text = controllers_yaml_path
    # The controller_manager embedded in this plugin does not read the URDF from
    # the SDF -- it waits for a std_msgs/String on the `robot_description` TOPIC
    # ("Waiting for data on 'robot_description' topic to finish initialization")
    # and never activates a controller without it. Since robot_state_publisher
    # below publishes on /leeloo/robot_description like the rest of the stack,
    # the embedded node needs the matching remap. This is the same remap the real
    # robot applies to its ros2_control_node in leeloo/launch/control.launch.py.
    ros_el = ET.SubElement(plugin_el, "ros")
    ET.SubElement(ros_el, "remapping").text = \
        "robot_description:=/leeloo/robot_description"

    # Holonomic base drive -- see module docstring for why this is a Gazebo
    # system plugin instead of a ros2_control mecanum_drive_controller.
    # Topic/frame names are explicit rather than left to the plugin's defaults so
    # the ros_gz_bridge arguments below have a fixed, documented target.
    #
    # This is VelocityControl, NOT MecanumDrive, and the distinction is the whole
    # point. MecanumDrive is not kinematic despite what an earlier version of
    # this docstring claimed: it converts the body twist into four WHEEL JOINT
    # velocities and then leaves it to wheel-ground contact to turn those into
    # body motion. leeloo.urdf's wheels are plain cylinders with isotropic
    # friction and no roller geometry, so that contact can only ever produce
    # longitudinal force -- exactly the limitation that ruled out the
    # ros2_control controller. Swapping one for the other changed nothing.
    # Measured on this install with the model's true pose from the physics
    # engine (NOT the plugin's odometry): commanding linear.y = 0.2 moved the
    # base +1.391 m FORWARD and 0.000 m sideways.
    #
    # The failure was invisible because MecanumDrive derives its odometry by
    # integrating the same kinematic model it used to compute the wheel speeds,
    # so /odometry and the odom->base_link TF confidently reported the perfect
    # strafe that never physically happened.
    #
    # VelocityControl applies the twist to the model directly, which is the
    # abstraction the real Clearpath firmware actually provides. Cost of this
    # choice, both real: the wheels no longer spin (nothing drives those joints
    # now, so it is purely cosmetic), and the base no longer interacts with the
    # ground through friction -- it will not slip, and it ignores slopes and
    # surface properties.
    vel_el = ET.SubElement(gazebo_el, "plugin", {
        "filename": "gz-sim-velocity-control-system",
        "name": "gz::sim::systems::VelocityControl",
    })
    ET.SubElement(vel_el, "topic").text = "cmd_vel"

    # Honest odometry: derived from the model's real pose in the world rather
    # than from a command-side kinematic model, so it cannot drift away from
    # what the physics engine actually did. dimensions=2 keeps it planar, which
    # is what the real base reports.
    odom_el = ET.SubElement(gazebo_el, "plugin", {
        "filename": "gz-sim-odometry-publisher-system",
        "name": "gz::sim::systems::OdometryPublisher",
    })
    ET.SubElement(odom_el, "odom_topic").text = "odometry"
    ET.SubElement(odom_el, "tf_topic").text = "tf"
    ET.SubElement(odom_el, "odom_frame").text = "odom"
    ET.SubElement(odom_el, "robot_base_frame").text = "base_link"
    ET.SubElement(odom_el, "odom_publish_frequency").text = "50"
    ET.SubElement(odom_el, "dimensions").text = "2"

    # Depth cameras. Neither exists in leeloo.urdf, so both are injected here.
    # update_rate matches frame_rate_hz in leeloo/config/cameras.yaml, which
    # curobo uses to normalise its TSDF decay -- changing one without the other
    # silently drifts the voxel map's memory.
    if "kinect" in cameras:
        # Azure Kinect. The real pose comes from hand-eye calibration
        # (kinect_tf_computation_node replaying kinect_hand_eye_result.yaml); in
        # sim it is a NOMINAL pose on camera_mount, tunable via launch args.
        # FOV/range approximate the depth_to_rgb stream (depth registered into
        # the 720P colour frame): the simulated sensor returns valid depth across
        # the whole colour FOV, whereas the real NFOV_UNBINNED depth FOV is
        # narrower (75x65 deg), so sim coverage is slightly optimistic.
        _add_camera(
            root, "kinect", "camera_mount",
            camera_poses["kinect_xyz"], camera_poses["kinect_rpy"],
            optical_frame="rgb_camera_link", gz_topic="kinect/depth",
            horizontal_fov=1.5708, near_clip=0.5, far_clip=3.86, update_rate=5)

    if "oak" in cameras:
        # OAK-D SR, eye-in-hand on the Doosan wrist. Offsets default to the same
        # values cameras.launch.py passes to the depthai driver. Approximation:
        # the driver's internal chain (oak base frame -> left camera optical
        # frame, a couple of cm) is not modelled -- the offset is applied
        # straight to the body link.
        _add_camera(
            root, "oak", "dsr01/link_6",
            camera_poses["oak_xyz"], camera_poses["oak_rpy"],
            optical_frame="oak_left_camera_optical_frame",
            gz_topic="oak/stereo", horizontal_fov=1.5620,
            near_clip=0.2, far_clip=1.5, update_rate=5)

    return ET.tostring(root, encoding="unicode")


def _setup(context, *args, **kwargs):
    """Resolved-argument half of the launch description.

    An OpaqueFunction is needed because the robot_description is built from
    launch arguments (which cameras, their poses), and those are only resolvable
    once the context exists.
    """
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    leeloo_gazebo_share = get_package_share_directory("leeloo_gazebo")
    controllers_yaml = os.path.join(
        leeloo_gazebo_share, "config", "leeloo_controllers.yaml")

    cameras = [c.strip() for c in arg("enabled_cameras").split(",") if c.strip()]
    camera_poses = {
        "kinect_xyz": arg("kinect_xyz"),
        "kinect_rpy": arg("kinect_rpy"),
        "oak_xyz": arg("oak_xyz"),
        "oak_rpy": arg("oak_rpy"),
    }
    use_sim_time = arg("use_sim_time").lower() in ("true", "1", "yes")
    standalone = arg("standalone").lower() in ("true", "1", "yes")

    robot_description = ParameterValue(
        _build_sim_robot_description(controllers_yaml, cameras, camera_poses),
        value_type=str)

    # sdformat's URDF->SDF conversion rewrites every mesh's "package://<pkg>/..."
    # URI to "model://<pkg>/..." verbatim (it does not resolve it via ament).
    # Gazebo then resolves "model://leeloo/meshes/x.stl" by searching
    # GZ_SIM_RESOURCE_PATH for a "leeloo" subdirectory -- neither leeloo nor
    # robotiq_description declare a <gazebo_ros gazebo_model_path> export in
    # their package.xml (they are not Gazebo packages), so without this their
    # share dirs are never on that path and every single mesh fails to load.
    # Point GZ_SIM_RESOURCE_PATH at the parent of each package's share/<pkg> dir.
    mesh_resource_path = os.pathsep.join([
        os.path.dirname(get_package_share_directory("leeloo")),
        os.path.dirname(get_package_share_directory("robotiq_description")),
    ])
    gz_resource_path_env = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        os.pathsep.join([
            mesh_resource_path,
            os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
        ]),
    )

    # This machine is a PRIME/Optimus laptop: the AMD iGPU drives the X
    # screen and the NVIDIA GPU is offload-only (confirmed via `xrandr
    # --listproviders`: NVIDIA-G0 is Sink Output, not Source Output). Without
    # __NV_PRIME_RENDER_OFFLOAD=1, forcing __GLX_VENDOR_LIBRARY_NAME=nvidia
    # makes the NVIDIA GLX vendor lib try to create a context directly on a
    # screen it doesn't own, which aborts gz's GUI process with an NV-GLX
    # BadValue X error. Both variables together route rendering to the
    # NVIDIA GPU via reverse PRIME instead of falling back to Mesa/radeonsi,
    # which is what keeps depth camera (kinect/oak) rendering -- the
    # dominant cost in this scene -- fast enough for RTF ~= 1.
    gl_vendor_env = SetEnvironmentVariable("__GLX_VENDOR_LIBRARY_NAME", "nvidia")
    gl_offload_env = SetEnvironmentVariable("__NV_PRIME_RENDER_OFFLOAD", "1")

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"),
                         "launch", "gz_sim.launch.py")),
        launch_arguments={
            "gz_args": [LaunchConfiguration("world"), " -r -v 3"]}.items(),
    )

    # Sim clock. Every node below runs on it: Gazebo's sim time starts near 0
    # while the wall clock is ~1.7e9, so mixing the two makes every TF lookup
    # fail. Note this does NOT make the execution chain sim-time-paced --
    # curobo's MPC servo loop, JointSpeedStrategy and execute_trajectory's timer
    # are all wall-clock based, so faithful timing still requires RTF ~= 1.
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    # MecanumDrive's <topic>/<odom_topic>/<tf_topic> are NOT model-scoped by
    # gz-sim: confirmed with `gz topic -l` that a relative name there resolves to
    # a plain global gz-transport topic ("cmd_vel" -> "/cmd_vel"), not
    # "/model/leeloo/cmd_vel" the way the DiffDrive plugin's own default would.
    # Bridging the model-scoped guess creates a dangling topic nobody on the gz
    # side touches, and commands vanish with no error at all.
    #
    # ROS-side names are remapped to the ones the real base uses, so the TF
    # relays already in leeloo's tf.launch.py (relay_ridgeback_tf) pick them up
    # unchanged. base_cmd_vel must NOT land on /cmd_vel: ridgeback_control
    # subscribes there and would be fed its own output.
    base_cmd_vel_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="base_cmd_vel_bridge",
        arguments=["/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist"],
        remappings=[("/cmd_vel", "/leeloo_gazebo/base_cmd_vel")],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )
    base_odom_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="base_odom_bridge",
        arguments=["/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry"],
        remappings=[("/odometry", "/r100_0597/odom")],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )
    base_tf_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="base_tf_bridge",
        arguments=["/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V"],
        remappings=[("/tf", "/r100_0597/tf")],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    # gz-sensors derives the camera_info topic by REPLACING the last segment of
    # <topic>, not by appending to it: <topic>kinect/depth</topic> publishes the
    # image on /kinect/depth and the info on /kinect/camera_info -- NOT
    # /kinect/depth/camera_info. Bridging the appended name creates a subscriber
    # with no publisher, and camera_info never arrives; curobo then cannot
    # unproject the depth image and the ESDF stays empty, with no error anywhere.
    # Confirmed with `gz topic -i -t <name>` (publishers vs subscribers) -- worth
    # redoing that check for any camera added here.
    camera_bridges = []
    if "kinect" in cameras:
        camera_bridges.append(Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="kinect_depth_bridge",
            arguments=[
                "/kinect/depth@sensor_msgs/msg/Image[gz.msgs.Image",
                "/kinect/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            ],
            remappings=[
                ("/kinect/depth", "/depth_to_rgb/image_raw"),
                ("/kinect/camera_info", "/depth_to_rgb/camera_info"),
            ],
            parameters=[{"use_sim_time": use_sim_time}],
            output="screen",
        ))
    if "oak" in cameras:
        camera_bridges.append(Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="oak_depth_bridge",
            arguments=[
                "/oak/stereo@sensor_msgs/msg/Image[gz.msgs.Image",
                "/oak/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            ],
            remappings=[
                ("/oak/stereo", "/oak/stereo/image_raw"),
                ("/oak/camera_info", "/oak/stereo/camera_info"),
            ],
            parameters=[{"use_sim_time": use_sim_time}],
            output="screen",
        ))

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="leeloo_sim_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": use_sim_time}],
        remappings=[("robot_description", "/leeloo/robot_description")],
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        parameters=[{
            "topic": "/leeloo/robot_description",
            "name": "leeloo",
            "z": LaunchConfiguration("spawn_z"),
            "use_sim_time": use_sim_time,
        }],
    )

    dsr_sim_bridge = Node(
        package="leeloo_gazebo",
        executable="dsr_sim_bridge",
        name="dsr_sim_bridge",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )
    base_sim_bridge = Node(
        package="leeloo_gazebo",
        executable="base_sim_bridge",
        name="base_sim_bridge",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Standalone convenience only: with the full bringup (sim:=true),
    # control.launch.py already runs ridgeback_control and passing
    # standalone:=false avoids a second instance.
    standalone_nodes = []
    if standalone:
        standalone_nodes.append(Node(
            package="leeloo",
            executable="ridgeback_control",
            name="ridgeback_control",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        ))

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )
    arm_velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_velocity_controller"],
    )
    # Inactive: claims the same joints as arm_velocity_controller. Activate by
    # hand (`ros2 control switch_controllers`) for a position-mode test without
    # curobo.
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--inactive"],
    )
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robotiq_gripper_controller"],
    )

    # gz_ros2_control's controller_manager only comes alive once Gazebo has
    # loaded the model, so the spawners are chained off spawn_entity's exit
    # rather than started unconditionally at launch time.
    delayed_joint_state_broadcaster = RegisterEventHandler(
        OnProcessExit(target_action=spawn_entity,
                      on_exit=[joint_state_broadcaster_spawner]))
    delayed_controllers = RegisterEventHandler(
        OnProcessExit(target_action=joint_state_broadcaster_spawner, on_exit=[
            arm_velocity_controller_spawner,
            arm_controller_spawner,
            gripper_controller_spawner,
        ]))

    # Everything that touches TF goes on /leeloo/tf[_static] like the real robot
    # (leeloo/launch/tf.launch.py uses the same GroupAction + SetRemap pattern),
    # so RViz configs and curobo's TF lookups need no remapping in sim.
    tf_scoped = GroupAction([
        SetRemap("/tf", LaunchConfiguration("tf_remap")),
        SetRemap("/tf_static", LaunchConfiguration("tf_static_remap")),
        robot_state_publisher,
    ])

    return [
        gz_resource_path_env,
        gl_vendor_env,
        gl_offload_env,
        gz_sim,
        clock_bridge,
        base_cmd_vel_bridge,
        base_odom_bridge,
        base_tf_bridge,
        *camera_bridges,
        tf_scoped,
        dsr_sim_bridge,
        base_sim_bridge,
        *standalone_nodes,
        spawn_entity,
        delayed_joint_state_broadcaster,
        delayed_controllers,
    ]


def generate_launch_description():
    default_world = os.path.join(
        get_package_share_directory("leeloo_gazebo"), "worlds", "leeloo_world.sdf")

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value=default_world),
        # base_link rests at wheel_radius (0.0759) - wheel z-offset (0.05)
        # = 0.0259 m; a hair above that avoids both an initial drop and an
        # initial ground interpenetration.
        DeclareLaunchArgument("spawn_z", default_value="0.03"),

        DeclareLaunchArgument("tf_remap", default_value="/leeloo/tf"),
        DeclareLaunchArgument("tf_static_remap", default_value="/leeloo/tf_static"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),

        DeclareLaunchArgument(
            "standalone", default_value="true",
            description="Also run leeloo's ridgeback_control so /cmd_vel works "
                        "without the full bringup. Set false when included from "
                        "leeloo's control.launch.py, which runs it already."),

        # Same comma-separated convention as leeloo/launch/cameras.launch.py.
        # Simulating both 1280x720 depth cameras is the dominant cost in the
        # scene: drop to '' for MPC timing tests, where RTF ~= 1 matters more.
        DeclareLaunchArgument(
            "enabled_cameras", default_value="kinect,oak",
            description="Simulated depth cameras, comma-separated "
                        "('kinect', 'oak', 'kinect,oak', or '' for none)."),

        # NOMINAL Kinect pose on camera_mount (which sits at base_link
        # (0.152, -0.360, 0.280)). Aims the optical axis at roughly (0.9, 0, 0.9)
        # in base_link, i.e. the middle of the arm's workspace, with a slight
        # downward tilt. This is a placeholder for the hand-eye calibration
        # result, not a measurement -- adjust freely.
        DeclareLaunchArgument("kinect_xyz", default_value="0 0 0.6"),
        DeclareLaunchArgument("kinect_rpy", default_value="0 0.1 0.449"),

        # OAK-D SR eye-in-hand offsets off dsr01/link_6 -- same defaults
        # cameras.launch.py passes to the depthai driver.
        DeclareLaunchArgument(
            "oak_xyz",
            default_value="0.0924476130 -0.01320633498 0.024658523349"),
        DeclareLaunchArgument(
            "oak_rpy",
            default_value="3.114168001 -1.323787043 -3.113287621"),

        OpaqueFunction(function=_setup),
    ])
