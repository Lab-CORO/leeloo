#!/usr/bin/env python3
"""Grasp an object whose pose is given by a fiducial (ArTag) marker.

Same marker-tracking + TF logic as brain.py, but instead of computing a
reachability score it performs a simple pick, driven by an explicit
event-driven state machine:

  idle --user_trigger--> act_move_pregrasp
  act_move_pregrasp --arm_done--> act_open_gripper
  act_open_gripper --gripper_open--> srv_mask_object
  srv_mask_object --object_masked--> act_move_grasp
  act_move_grasp --arm_done--> act_close_gripper
  act_close_gripper --gripper_closed--> act_retract_arm
  act_retract_arm --arm_done--> idle
  (any act_*/srv_* state) --failure--> idle

`self.current_state` doubles as the busy flag: only `idle` accepts
`user_trigger`, so a trigger received mid-sequence is ignored. Adding a new
state later = one constant, one `_on_enter_*` handler, one entry in
`_state_handlers`, one or two entries in `TRANSITIONS`.
"""

from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy

import tf2_ros
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from tf2_geometry_msgs import do_transform_pose_stamped

from geometry_msgs.msg import PoseStamped, Pose, TransformStamped, Vector3
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
from control_msgs.action import ParallelGripperCommand
from curobo_msgs.action import SendTrajectory
from curobo_msgs.srv import SetMask
from ros2_markertracker_interfaces.msg import FiducialMarkerArray
from heron_interface.action import TriggerGrasp
from dsr_msgs2.srv import MoveLine, TaskComplianceCtrl, ReleaseComplianceCtrl, CheckMotion

from rclpy.action import ActionServer, CancelResponse

# Doosan dsr_msgs2 enums (see motion_services.html#moveline and
# force_services.html#taskcompliancectrl) — hardcoded here rather than
# pulled from generated message constants, since it's unconfirmed whether
# this driver version exposes them as such; adjust if it does.
DR_BASE = 0
DR_TOOL = 1
DR_WORLD = 2
MOVE_MODE_RELATIVE = 1
SYNC = 0
# CheckMotion.status
DR_STATE_IDLE = 0
DR_STATE_INIT = 1
DR_STATE_BUSY = 2


# States (plain strings, not Enum: a single place to touch when adding one).
IDLE = 'idle'
ACT_MOVE_PREGRASP = 'act_move_pregrasp'
ACT_OPEN_GRIPPER = 'act_open_gripper'
SRV_MASK_OBJECT = 'srv_mask_object'
ACT_MOVE_GRASP = 'act_move_grasp'
ACT_CLOSE_GRIPPER = 'act_close_gripper'
ACT_RETRACT_ARM = 'act_retract_arm'

_BUSY_STATES = (
    ACT_MOVE_PREGRASP, ACT_OPEN_GRIPPER, SRV_MASK_OBJECT,
    ACT_MOVE_GRASP, ACT_CLOSE_GRIPPER, ACT_RETRACT_ARM,
)

TRANSITIONS = {
    (IDLE, 'user_trigger'): ACT_MOVE_PREGRASP,
    (ACT_MOVE_PREGRASP, 'arm_done'): ACT_OPEN_GRIPPER,
    (ACT_OPEN_GRIPPER, 'gripper_open'): SRV_MASK_OBJECT,
    (SRV_MASK_OBJECT, 'object_masked'): ACT_MOVE_GRASP,
    (ACT_MOVE_GRASP, 'arm_done'): ACT_CLOSE_GRIPPER,
    (ACT_CLOSE_GRIPPER, 'gripper_closed'): ACT_RETRACT_ARM,
    (ACT_RETRACT_ARM, 'arm_done'): IDLE,
    # Generic abort: any busy state can bail out back to idle.
    **{(s, 'failure'): IDLE for s in _BUSY_STATES},
}


class ObjectGrasper(Node):
    """Pick an ArTag-defined object via an explicit state machine."""

    def __init__(self):
        super().__init__('object_grasper')

        self._load_parameters()

        # Latest (averaged) fiducial marker pose (in its own camera frame), or None.
        self._marker_pose = None
        # Recent accepted (position, quaternion, stamp_seconds) samples feeding
        # that average — see on_markers.
        self._marker_pose_buffer = deque(maxlen=self.marker_avg_window)

        self._setup_interfaces()
        self._setup_state_machine()

        self.get_logger().info("Object grasper initialized")
        self.get_logger().info(f"  - Marker topic: {self.marker_topic} (id={self.target_marker_id})")
        self.get_logger().info(
            f"  - Marker pose averaging: window={self.marker_avg_window}, "
            f"max_jump={self.marker_avg_max_jump:.3f}m, reset_gap={self.marker_avg_reset_gap:.1f}s")
        self.get_logger().info(f"  - Trigger topic: {self.trigger_topic}")
        self.get_logger().info(f"  - Planning frame: {self.planning_frame}")
        self.get_logger().info(f"  - Execute action: {self.execute_action}")
        self.get_logger().info(
            f"  - MPC live-goal topic: {self.mpc_goal_topic} (rate={self.mpc_goal_rate:.1f}Hz)")
        self.get_logger().info(f"  - State topic: {self.state_topic} (std_msgs/String, latched)")
        self.get_logger().info(
            f"  - Compliant grasp approach: task_compliance={self.task_compliance_ctrl_service}, "
            f"move_line={self.move_line_service}, "
            f"release_compliance={self.release_compliance_ctrl_service}, "
            f"check_motion={self.check_motion_service}, "
            f"stiffness=[{self.compliant_translation_stiffness:.0f}N/m, "
            f"{self.compliant_rotation_stiffness:.0f}Nm/rad], ref={self.grasp_move_ref} "
            f"(base-frame approach vector), "
            f"settle={self.compliance_settle_time:.2f}s before MoveLine")
        self.get_logger().info(
            f"  - RT streaming toggle: {self.enable_streaming_service} "
            f"(paused during compliance calls)")
        self.get_logger().info(
            f"  - Doosan service call timeout: {self.doosan_service_timeout:.1f}s "
            f"(move_line: {self.move_line_timeout:.1f}s — blocking motion call)")
        self.get_logger().info(
            f"  - Lift: move_line ref={self.lift_move_ref}, +{self.lift_distance:.3f}m")
        self.get_logger().info(
            f"  - Gripper action: {self.gripper_action} (joint={self.gripper_joint_name})")
        self.get_logger().info(f"  - Mask service: {self.mask_service_name}")
        self.get_logger().info(
            f"  - Cube offset from tag: {self.grasp_cube_distance:.3f}m "
            f"along local {self.grasp_cube_axis}-axis")

    def _load_parameters(self):
        self.declare_parameter('marker_topic', '/fiducial_markers')
        self.declare_parameter('target_marker_id', -1)
        self.declare_parameter('trigger_topic', '/brain/trigger')

        # Marker pose smoothing: average the last N detections instead of
        # trusting each one raw, so a single bad detection frame doesn't
        # instantly become the grasp target.
        self.declare_parameter('marker_avg_window', 5)
        # A new sample farther than this from the current running average is
        # dropped as a likely detection glitch rather than real marker motion.
        self.declare_parameter('marker_avg_max_jump', 0.05)  # m
        # ...unless the marker hasn't been seen for longer than this: then the
        # buffer is stale and the new sample is accepted unconditionally as a
        # fresh acquisition instead of being compared against it.
        self.declare_parameter('marker_avg_reset_gap', 1.0)  # s

        # Planning frame = curobo base_link (target_pose has no frame_id).
        self.declare_parameter('planning_frame', 'dsr01/world')
        # Unified "plan + execute" action: moves the arm to target_pose.
        self.declare_parameter('execute_action', '/curobo_trajectory_planner/execute_trajectory')
        # While execute_trajectory's reactive controller (MPC) is servoing toward
        # pregrasp/grasp, republish the offset target recomputed from the *live*
        # marker pose here so a moving tag keeps pulling the goal (mpc_goal_callback
        # forwards it into planner.set_live_goal(...)).
        self.declare_parameter('mpc_goal_topic', '/curobo_trajectory_planner/mpc_goal')
        self.declare_parameter('mpc_goal_rate', 10.0)
        # Current state-machine state (e.g. "idle", "act_move_grasp"), as
        # std_msgs/String. Transient-local + published on every transition, so
        # a subscriber started mid-sequence still gets the current value
        # immediately instead of waiting for the next change.
        self.declare_parameter('state_topic', '~/state')

        # act_move_grasp (final pregrasp->grasp leg, right before contact) uses
        # the Doosan driver's native compliance instead of curobo, so
        # marker/pose error at contact is absorbed by compliance rather than
        # fought by the free-space MPC: TaskComplianceCtrl (soft), then a
        # normal MoveLine toward the object while compliant, then
        # ReleaseComplianceCtrl. An earlier attempt saw movel rejected while
        # TASK_COMPLIANCE_CONTROL was active ("state[TASK_COMPLIANCE_CONTROL]
        # rejected event[eSpeedJ]"), but that was before RT streaming was
        # disabled first (see enable_streaming_service below) — the rejected
        # event may have been the streaming loop's own commands rather than
        # movel itself. Verify on the robot; if movel is still rejected under
        # compliance even with streaming off, this needs a different approach
        # (e.g. SetDesiredForce) after all.
        self.declare_parameter('move_line_service', '/dsr01/dsr_controller2/motion/move_line')
        self.declare_parameter('task_compliance_ctrl_service', '/dsr01/dsr_controller2/force/task_compliance_ctrl')
        self.declare_parameter('release_compliance_ctrl_service', '/dsr01/dsr_controller2/force/release_compliance_ctrl')
        # Polled before ReleaseComplianceCtrl and before reporting the grasp
        # complete, so neither happens while the arm is still moving.
        self.declare_parameter('check_motion_service', '/dsr01/dsr_controller2/motion/check_motion')
        # dsr_controller2's RT command streaming must be paused before any
        # compliance call: left on, it keeps streaming commands (observed
        # rejection: internal event 'eSpeedJ') that TASK_COMPLIANCE_CONTROL
        # rejects, which faulted the controller into MOTION_HOLD.
        self.declare_parameter('enable_streaming_service', '/ExecuteTrajectory/enable_streaming')
        # Hard timeout on every individual Doosan service call: the driver was
        # observed to never reply at all to a rejected/invalid command, hanging
        # the await forever otherwise — this bounds every single call so the
        # state machine can always fail back to idle instead of freezing until
        # an external cancel.
        self.declare_parameter('doosan_service_timeout', 2.0)  # s
        # MoveLine needs its own, much longer bound: with sync_type=SYNC the
        # service can only reply once the motion physically finishes (confirmed
        # outside compliance: a 100mm@50mm/s lift takes >2s and tripped
        # doosan_service_timeout even though the move itself was running fine).
        # Must exceed the longest commanded move's duration, not the driver's
        # round-trip latency.
        self.declare_parameter('move_line_timeout', 20.0)  # s
        # check_motion poll period + overall bound on waiting for the robot to
        # come back to DR_STATE_IDLE. Belt-and-braces after MoveLine, which is
        # expected to have blocked already.
        self.declare_parameter('motion_poll_period', 0.1)   # s
        self.declare_parameter('motion_idle_timeout', 20.0)  # s
        # Soft on all three translations, default-stiff on rotation (matches
        # TaskComplianceCtrl's own rotational default of 200 Nm/rad).
        self.declare_parameter('compliant_translation_stiffness', 500.0)  # N/m, range 0-20000
        self.declare_parameter('compliant_rotation_stiffness', 200.0)     # Nm/rad, range 0-400
        self.declare_parameter('compliance_ramp_time', 0.4)               # s, range 0-1.0
        # MANDATORY dead time between task_compliance_ctrl and the MoveLine:
        # a movel issued while the stiffness is still ramping is ACCEPTED
        # (success=True) and then silently dropped — check_motion never leaves
        # IDLE and the robot doesn't move at all. Reproduced standalone: same
        # command, gap=0 -> movel replies in 0.20s and nothing moves; gap=1.0s
        # -> movel blocks 3.96s and the move runs, at both 500 and 3000 N/m.
        # Must be > compliance_ramp_time; 1.0s is the verified value (0.4s
        # alone was never tested and has no margin).
        self.declare_parameter('compliance_settle_time', 1.0)             # s
        # Reference frame for both TaskComplianceCtrl and the grasp MoveLine:
        # base-frame, matching the explicit approach->grasp direction vector
        # computed in _on_enter_move_pregrasp (_run['approach_dir']). Assumes
        # planning_frame is the controller's DR_BASE frame; switch to
        # DR_WORLD(2) if your controller (M2.40+) defines a separate world
        # frame that planning_frame actually matches.
        self.declare_parameter('grasp_move_ref', DR_BASE)
        self.declare_parameter('compliant_move_vel', [30.0, 20.0])  # [mm/s, deg/s]
        self.declare_parameter('compliant_move_acc', [60.0, 40.0])  # [mm/s^2, deg/s^2]

        # act_retract_arm (post-grasp lift) also uses MoveLine directly instead of
        # curobo. Not compliance-wrapped: this leaves contact and moves through
        # free space, so no need to absorb force error. ref=DR_BASE assumes
        # planning_frame ('dsr01/world') is the robot's own base frame with Z up;
        # switch to DR_WORLD if your controller (M2.40+) defines a separate world
        # frame that this TF frame actually corresponds to.
        self.declare_parameter('lift_move_ref', DR_BASE)
        self.declare_parameter('lift_move_vel', [50.0, 30.0])  # [mm/s, deg/s]
        self.declare_parameter('lift_move_acc', [80.0, 50.0])  # [mm/s^2, deg/s^2]

        # Gripper (real controller serves control_msgs/action/ParallelGripperCommand,
        # a sensor_msgs/JointState-based goal — NOT the older GripperCommand type).
        self.declare_parameter('gripper_action', '/robotiq_gripper_controller/gripper_cmd')
        self.declare_parameter('gripper_joint_name', 'robotiq_85_left_knuckle_joint')
        self.declare_parameter('gripper_open_position', 0.01)
        # URDF joint limit is [0.0, 0.8]; 0.7929 matches the driver's own
        # fully-closed default (2f_85.ros2_control.xacro).
        self.declare_parameter('gripper_closed_position', 0.79)
        # Gripper not wired yet: just log open/close instead of calling the action.
        self.declare_parameter('simulate_gripper', False)

        # Object masking: excludes the grasped object's known geometry from
        # world collision (curobo_ros robot_segmentation's set_mask service)
        # so the planner can enter its volume between mask and grasp.
        self.declare_parameter('mask_service_name', '/set_mask')
        self.declare_parameter('mask_object_name', 'grasp_target')
        self.declare_parameter('mask_object_dimensions', [0.1, 0.1, 0.1])

        # Approach geometry: offsets along the marker Z axis (metres).
        # position = marker_pos + approach_axis_sign * distance * marker_Z
        self.declare_parameter('approach_distance', -0.2)
        self.declare_parameter('grasp_distance', -0.15)
        # -1.0: approach from the -Z side so the gripper Z (aligned with tag Z)
        # points toward the object. Flip to +1.0 if the tag Z points the other way.
        self.declare_parameter('approach_axis_sign', -1.0)
        # Vertical (world/planning-frame Z) lift distance for act_retract_arm.
        self.declare_parameter('lift_distance', 0.10)
        # The cube is not exactly at the tag: it sits `grasp_cube_distance`
        # away from the tag along the tag's own local `grasp_cube_axis` axis
        # ('x', 'y', or 'z'). Tune both empirically by watching the
        # 'cube_target' TF frame (published alongside approach/grasp/lift)
        # against the real cube position in RViz.
        self.declare_parameter('grasp_cube_distance', 0.07)
        self.declare_parameter('grasp_cube_axis', 'x')

        self.marker_topic = self.get_parameter('marker_topic').value
        self.target_marker_id = self.get_parameter('target_marker_id').value
        self.trigger_topic = self.get_parameter('trigger_topic').value
        self.marker_avg_window = self.get_parameter('marker_avg_window').value
        self.marker_avg_max_jump = self.get_parameter('marker_avg_max_jump').value
        self.marker_avg_reset_gap = self.get_parameter('marker_avg_reset_gap').value
        self.planning_frame = self.get_parameter('planning_frame').value
        self.execute_action = self.get_parameter('execute_action').value
        self.mpc_goal_topic = self.get_parameter('mpc_goal_topic').value
        self.state_topic = self.get_parameter('state_topic').value
        self.mpc_goal_rate = self.get_parameter('mpc_goal_rate').value
        self.move_line_service = self.get_parameter('move_line_service').value
        self.task_compliance_ctrl_service = self.get_parameter('task_compliance_ctrl_service').value
        self.release_compliance_ctrl_service = self.get_parameter('release_compliance_ctrl_service').value
        self.check_motion_service = self.get_parameter('check_motion_service').value
        self.enable_streaming_service = self.get_parameter('enable_streaming_service').value
        self.doosan_service_timeout = self.get_parameter('doosan_service_timeout').value
        self.move_line_timeout = self.get_parameter('move_line_timeout').value
        self.motion_poll_period = self.get_parameter('motion_poll_period').value
        self.motion_idle_timeout = self.get_parameter('motion_idle_timeout').value
        self.compliant_translation_stiffness = self.get_parameter('compliant_translation_stiffness').value
        self.compliant_rotation_stiffness = self.get_parameter('compliant_rotation_stiffness').value
        self.compliance_ramp_time = self.get_parameter('compliance_ramp_time').value
        self.compliance_settle_time = self.get_parameter('compliance_settle_time').value
        self.grasp_move_ref = self.get_parameter('grasp_move_ref').value
        self.compliant_move_vel = self.get_parameter('compliant_move_vel').value
        self.compliant_move_acc = self.get_parameter('compliant_move_acc').value
        self.lift_move_ref = self.get_parameter('lift_move_ref').value
        self.lift_move_vel = self.get_parameter('lift_move_vel').value
        self.lift_move_acc = self.get_parameter('lift_move_acc').value
        self.gripper_action = self.get_parameter('gripper_action').value
        self.gripper_joint_name = self.get_parameter('gripper_joint_name').value
        self.gripper_open_position = self.get_parameter('gripper_open_position').value
        self.gripper_closed_position = self.get_parameter('gripper_closed_position').value
        self.simulate_gripper = self.get_parameter('simulate_gripper').value
        self.mask_service_name = self.get_parameter('mask_service_name').value
        self.mask_object_name = self.get_parameter('mask_object_name').value
        self.mask_object_dimensions = self.get_parameter('mask_object_dimensions').value
        self.approach_distance = self.get_parameter('approach_distance').value
        self.grasp_distance = self.get_parameter('grasp_distance').value
        self.approach_axis_sign = self.get_parameter('approach_axis_sign').value
        self.lift_distance = self.get_parameter('lift_distance').value
        self.grasp_cube_distance = self.get_parameter('grasp_cube_distance').value
        self.grasp_cube_axis = self.get_parameter('grasp_cube_axis').value

    def _setup_interfaces(self):
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.marker_sub = self.create_subscription(
            FiducialMarkerArray, self.marker_topic, self.on_markers, 10
        )

        self.execute_client = ActionClient(self, SendTrajectory, self.execute_action)
        self.gripper_client = ActionClient(self, ParallelGripperCommand, self.gripper_action)
        self.mask_client = self.create_client(SetMask, self.mask_service_name)

        # Own ReentrantCallbackGroup for every Doosan native service client
        # plus the timers used to poll/timeout them: their done-callbacks (or
        # timeout timers) resolve Futures that action-callback coroutines
        # (_on_enter_move_grasp etc.) await. If these shared the action
        # server's default MutuallyExclusiveCallbackGroup, that await would
        # deadlock — the group won't start a new callback while the
        # (suspended-but-not-finished) action callback still occupies it.
        self._poll_callback_group = ReentrantCallbackGroup()

        # Doosan native services for the compliant grasp approach.
        self.move_line_client = self.create_client(
            MoveLine, self.move_line_service, callback_group=self._poll_callback_group)
        self.task_compliance_client = self.create_client(
            TaskComplianceCtrl, self.task_compliance_ctrl_service, callback_group=self._poll_callback_group)
        self.release_compliance_client = self.create_client(
            ReleaseComplianceCtrl, self.release_compliance_ctrl_service,
            callback_group=self._poll_callback_group)
        self.check_motion_client = self.create_client(
            CheckMotion, self.check_motion_service, callback_group=self._poll_callback_group)
        # Must be disabled before, and re-enabled after, every compliance
        # window — see enable_streaming_service above.
        self.enable_streaming_client = self.create_client(
            SetBool, self.enable_streaming_service, callback_group=self._poll_callback_group)

        # Live-tracking goal updates for the reactive controller while it servoes
        # (see mpc_goal_topic above).
        self.mpc_goal_pub = self.create_publisher(Pose, self.mpc_goal_topic, 10)

        # Transient-local so a late subscriber gets the current state
        # immediately without waiting for the next transition.
        state_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.state_pub = self.create_publisher(String, self.state_topic, state_qos)

        # Grasp trigger as an action (allows cancel, returns success/message).
        # cancel_callback is required: rclpy's default rejects every cancel
        # request, which would make cancellation silently do nothing.
        self.grasp_action_server = ActionServer(
            self, TriggerGrasp, 'grasp_trigger', self.grasp_trigger_cb,
            cancel_callback=self._on_grasp_cancel_requested)

        # Track in-flight goal handles so we can cancel them.
        self._trajectory_goal_handle = None
        self._gripper_goal_handle = None
        self._cancel_requested = False
        # Set by _on_execute_feedback once on_target=True is seen, so
        # execute_trajectory knows a cancel-on-success is in flight.
        self._on_target_reached = False

        # Broadcast the computed target poses as TF (approach/grasp/lift_target) so
        # the planned goal can be checked in RViz before triggering.
        self.tf_broadcaster = TransformBroadcaster(self)
        self.target_tf_timer = self.create_timer(0.1, self._publish_target_tf)

    def _setup_state_machine(self):
        self.current_state = IDLE
        # Run-scoped data (poses computed at trigger time), read by later
        # states so a moving marker mid-sequence can't shift the target.
        self._run = {}
        self._state_handlers = {
            ACT_MOVE_PREGRASP: self._on_enter_move_pregrasp,
            ACT_OPEN_GRIPPER: self._on_enter_open_gripper,
            SRV_MASK_OBJECT: self._on_enter_mask_object,
            ACT_MOVE_GRASP: self._on_enter_move_grasp,
            ACT_CLOSE_GRIPPER: self._on_enter_close_gripper,
            ACT_RETRACT_ARM: self._on_enter_retract_arm,
        }

    # ── Callbacks ────────────────────────────────────────────────────────────

    def on_markers(self, msg: FiducialMarkerArray):
        """Store a smoothed pose of the tracked marker (in its own camera
        frame): the mean of the last `marker_avg_window` accepted detections,
        rejecting any sample that jumps farther than `marker_avg_max_jump`
        from that running average — a single bad detection frame shouldn't
        instantly become the grasp target. If the marker hasn't been seen for
        longer than `marker_avg_reset_gap`, the buffer is stale and gets reset
        instead, so a real (large) change in marker position isn't mistaken
        for a glitch forever.
        """
        if not msg.marker:
            return

        selected = None
        if self.target_marker_id < 0:
            selected = msg.marker[0]
        else:
            for m in msg.marker:
                if m.id == self.target_marker_id:
                    selected = m
                    break

        if selected is None:
            return

        pose = PoseStamped()
        pose.header = selected.pose_cov_stamped.header
        pose.pose = selected.pose_cov_stamped.pose.pose
        stamp = pose.header.stamp.sec + pose.header.stamp.nanosec * 1e-9

        if self._marker_pose_buffer:
            last_stamp = self._marker_pose_buffer[-1][2]
            if stamp - last_stamp > self.marker_avg_reset_gap:
                self.get_logger().info(
                    f"Marker reacquired after {stamp - last_stamp:.2f}s gap, "
                    f"resetting pose average")
                self._marker_pose_buffer.clear()

        if self._marker_pose_buffer:
            avg_x, avg_y, avg_z = self._average_position(self._marker_pose_buffer)
            p = pose.pose.position
            jump = ((p.x - avg_x) ** 2 + (p.y - avg_y) ** 2 + (p.z - avg_z) ** 2) ** 0.5
            if jump > self.marker_avg_max_jump:
                self.get_logger().warn(
                    f"Marker detection jumped {jump:.3f}m from running average, "
                    f"discarding as a likely glitch")
                return

        self._marker_pose_buffer.append((
            (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z),
            (pose.pose.orientation.x, pose.pose.orientation.y,
             pose.pose.orientation.z, pose.pose.orientation.w),
            stamp,
        ))

        averaged = PoseStamped()
        averaged.header = pose.header  # latest frame_id + stamp
        ax, ay, az = self._average_position(self._marker_pose_buffer)
        aqx, aqy, aqz, aqw = self._average_quaternion(self._marker_pose_buffer)
        averaged.pose.position.x = ax
        averaged.pose.position.y = ay
        averaged.pose.position.z = az
        averaged.pose.orientation.x = aqx
        averaged.pose.orientation.y = aqy
        averaged.pose.orientation.z = aqz
        averaged.pose.orientation.w = aqw
        self._marker_pose = averaged

    @staticmethod
    def _average_position(buffer):
        n = len(buffer)
        sx = sum(sample[0][0] for sample in buffer)
        sy = sum(sample[0][1] for sample in buffer)
        sz = sum(sample[0][2] for sample in buffer)
        return (sx / n, sy / n, sz / n)

    @staticmethod
    def _average_quaternion(buffer):
        """Chordal mean: q and -q represent the same rotation, so flip each
        sample into the same hemisphere as the first before averaging
        component-wise and renormalizing. Good enough for the small
        orientation variation expected between consecutive detections of the
        same marker (not a substitute for proper SLERP-based averaging across
        large rotations)."""
        ref = buffer[0][1]
        sx = sy = sz = sw = 0.0
        for _, q, _ in buffer:
            if q[0] * ref[0] + q[1] * ref[1] + q[2] * ref[2] + q[3] * ref[3] < 0.0:
                q = (-q[0], -q[1], -q[2], -q[3])
            sx += q[0]
            sy += q[1]
            sz += q[2]
            sw += q[3]
        n = len(buffer)
        x, y, z, w = sx / n, sy / n, sz / n, sw / n
        norm = (x * x + y * y + z * z + w * w) ** 0.5
        if norm == 0.0:
            norm = 1.0
        return (x / norm, y / norm, z / norm, w / norm)

    def _on_grasp_cancel_requested(self, goal_handle):
        """Accept the cancel request and cancel whatever sub-goal is in flight.

        Cancelling the sub-goal makes the awaiting execute_trajectory/
        call_gripper_action return False, which the state machine already
        turns into a 'failure' transition back to IDLE — no separate abort
        path needed.
        """
        self.get_logger().info("Cancel requested, cancelling in-flight sub-goal(s)...")
        self._cancel_requested = True
        if self._trajectory_goal_handle is not None:
            self._trajectory_goal_handle.cancel_goal_async()
        if self._gripper_goal_handle is not None:
            self._gripper_goal_handle.cancel_goal_async()
        return CancelResponse.ACCEPT

    async def grasp_trigger_cb(self, goal_handle):
        """Action callback for grasp trigger. Supports cancel to stop the sequence."""
        if self.current_state != IDLE:
            self.get_logger().warn(
                f"Grasp sequence already running (state={self.current_state}), rejecting")
            goal_handle.abort()
            result = TriggerGrasp.Result()
            result.success = False
            result.message = "Sequence already running"
            return result

        self.get_logger().info("Trigger received, starting grasp sequence")
        self._cancel_requested = False

        try:
            await self.fire('user_trigger')
        except Exception as e:
            self.get_logger().error(f"Unexpected error in grasp sequence: {e}")
            self.current_state = IDLE
            goal_handle.abort()
            result = TriggerGrasp.Result()
            result.success = False
            result.message = str(e)
            return result

        result = TriggerGrasp.Result()
        if self._cancel_requested:
            self.get_logger().info("Grasp sequence cancelled")
            result.success = False
            result.message = "Cancelled"
            goal_handle.canceled()
            return result

        result.success = (self.current_state == IDLE)
        result.message = "Grasp sequence completed" if result.success else "Sequence failed"
        if result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    # ── State machine ────────────────────────────────────────────────────────

    @property
    def current_state(self) -> str:
        return self._current_state

    @current_state.setter
    def current_state(self, value: str) -> None:
        """Every write to current_state (init, fire(), the exception/cancel
        paths in grasp_trigger_cb) goes through here, so state_topic always
        reflects the actual state without each call site publishing by hand."""
        self._current_state = value
        self.state_pub.publish(String(data=value))

    async def fire(self, event: str) -> None:
        """Look up (current_state, event) and transition, running the next
        state's on-enter handler. Unknown event for the current state: log
        and no-op (never raises)."""
        key = (self.current_state, event)
        next_state = TRANSITIONS.get(key)
        if next_state is None:
            self.get_logger().warn(
                f"No transition for event '{event}' in state '{self.current_state}' — ignored")
            return
        self.get_logger().info(f"{self.current_state} --{event}--> {next_state}")
        self.current_state = next_state
        handler = self._state_handlers.get(next_state)
        if handler is not None:
            await handler()

    async def _on_enter_move_pregrasp(self) -> None:
        if self._marker_pose is None:
            self.get_logger().warn("No marker received yet, cannot grasp")
            await self.fire('failure')
            return
        marker_in_plan = self._transform_marker_to(self.planning_frame)
        if marker_in_plan is None:
            await self.fire('failure')
            return
        cube_in_plan = self._cube_pose_from_marker(marker_in_plan)

        approach_pose = self._offset_along_marker_z(cube_in_plan, self.approach_distance)
        grasp_pose = self._offset_along_marker_z(cube_in_plan, self.grasp_distance)
        self._run = {
            'marker_in_plan': marker_in_plan,
            'cube_in_plan': cube_in_plan,
            'approach_pose': approach_pose,
            # Unit vector approach->grasp, in the planning frame. act_move_grasp
            # moves along this via a base-frame relative MoveLine while
            # compliant (see grasp_move_ref), rather than assuming the tool's
            # +Z axis matches this direction.
            'approach_dir': self._unit_vector_between(approach_pose, grasp_pose),
        }

        self.get_logger().info("Moving to PREGRASP (approach) pose...")
        ok = await self.execute_trajectory(
            self._run['approach_pose'],
            live_pose_fn=lambda: self._recompute_live_pose(self.approach_distance))
        await self.fire('arm_done' if ok else 'failure')

    async def _on_enter_open_gripper(self) -> None:
        self.get_logger().info("Opening gripper...")
        ok = await self.call_gripper_action(self.gripper_open_position, grasp_object = False)
        await self.fire('gripper_open' if ok else 'failure')

    async def _on_enter_mask_object(self) -> None:
        self.get_logger().info(f"Masking object '{self.mask_object_name}'...")
        pose = self._run['cube_in_plan'].pose
        ok = await self.call_set_mask_service(pose)
        await self.fire('object_masked' if ok else 'failure')

    async def _on_enter_move_grasp(self) -> None:
        """Final pregrasp->grasp leg: TaskComplianceCtrl (soft), then a normal
        MoveLine toward the object along _run['approach_dir'] (a base-frame
        unit vector, computed in _on_enter_move_pregrasp) while compliant, so
        marker/pose error at contact is absorbed by compliance instead of
        fought by the free-space MPC. No SetDesiredForce: compliance alone
        plus a position move, nothing pushing with a target force.

        Ordering is load-bearing: the MoveLine must not be issued until the
        stiffness ramp has settled (compliance_settle_time), or it is accepted
        and silently dropped. Once it does run, sync_type=SYNC blocks normally
        under compliance — wait_motion_idle() after it is only a safety net,
        not the completion signal.

        dsr_controller2's RT command streaming is paused for the whole
        compliance window: left on, it keeps streaming commands that
        TASK_COMPLIANCE_CONTROL rejects (observed:
        "state[TASK_COMPLIANCE_CONTROL] rejected event[eSpeedJ]"), which
        faulted the controller into MOTION_HOLD. Note this is about the
        streaming loop's own commands, not movel: a plain movel runs fine with
        streaming left on (act_retract_arm does exactly that). Streaming is
        always re-enabled before leaving this state (success, failure, or
        exception) — act_retract_arm's MoveLine needs it back on.
        """
        self.get_logger().info("Moving to GRASP pose (compliant linear approach)...")
        approach_dir = self._run.get('approach_dir')
        if approach_dir is None:
            self.get_logger().error("No approach direction available, cannot move toward grasp")
            await self.fire('failure')
            return
        if not await self.call_enable_streaming(False):
            self.get_logger().error("Failed to disable RT streaming, aborting compliant approach")
            await self.fire('failure')
            return

        ok = False
        try:
            stx = (
                [self.compliant_translation_stiffness] * 3
                + [self.compliant_rotation_stiffness] * 3
            )
            ok = await self.call_task_compliance_ctrl(
                stx, ref=self.grasp_move_ref, ramp_time=self.compliance_ramp_time)

            if ok:
                # Do not remove: a MoveLine issued during the stiffness ramp is
                # accepted and then dropped — see compliance_settle_time.
                await self._sleep(self.compliance_settle_time)
                delta_mm = abs(
                    self.approach_axis_sign * (self.approach_distance - self.grasp_distance)) * 1000.0
                dx, dy, dz = (c * delta_mm for c in approach_dir)
                ok = await self.call_move_line_relative(
                    [dx, dy, dz, 0.0, 0.0, 0.0], ref=self.grasp_move_ref,
                    vel=self.compliant_move_vel, acc=self.compliant_move_acc)

            # Confirm the robot has actually stopped before releasing
            # compliance, on the failure path too: a motion may be running
            # regardless of what move_line reported.
            if not await self.wait_motion_idle():
                ok = False

            # Always try to release compliance — even on failure/cancel — so
            # the robot isn't left soft for the next (curobo) motion.
            if not await self.call_release_compliance_ctrl():
                self.get_logger().error("Failed to release compliance control!")
        finally:
            if not await self.call_enable_streaming(True):
                self.get_logger().error("Failed to re-enable RT streaming!")

        await self.fire('arm_done' if ok else 'failure')

    @staticmethod
    def _unit_vector_between(from_pose: Pose, to_pose: Pose):
        """Unit vector from one Pose's position to another's, in their shared
        (planning) frame. Returns None if the two coincide."""
        dx = to_pose.position.x - from_pose.position.x
        dy = to_pose.position.y - from_pose.position.y
        dz = to_pose.position.z - from_pose.position.z
        norm = (dx * dx + dy * dy + dz * dz) ** 0.5
        if norm == 0.0:
            return None
        return (dx / norm, dy / norm, dz / norm)

    async def _on_enter_close_gripper(self) -> None:
        self.get_logger().info("Closing gripper...")
        ok = await self.call_gripper_action(self.gripper_closed_position, grasp_object = True)
        await self.fire('gripper_closed' if ok else 'failure')

    async def _on_enter_retract_arm(self) -> None:
        """Post-grasp lift: a relative linear move (Doosan MoveLine) straight up
        along ref=lift_move_ref's Z axis, not curobo — this is free-space motion
        away from contact, so no compliance wrapper is needed."""
        self.get_logger().info(f"Retracting arm (+{self.lift_distance:.3f}m Z, linear move)...")
        ok = await self.call_move_line_relative(
            [0.0, 0.0, self.lift_distance * 1000.0, 0.0, 0.0, 0.0], ref=self.lift_move_ref,
            vel=self.lift_move_vel, acc=self.lift_move_acc)
        # Outside compliance move_line does appear to block, but don't rely on
        # it — report the grasp complete only once the arm has actually stopped.
        ok = await self.wait_motion_idle() and ok
        if ok:
            self.get_logger().info("Grasp sequence complete")
        await self.fire('arm_done' if ok else 'failure')

    # ── Helpers ──────────────────────────────────────────────────────────────

    def _transform_marker_to(self, target_frame) -> PoseStamped:
        """Transform the stored marker pose into target_frame. Returns PoseStamped or None."""
        source_frame = self._marker_pose.header.frame_id
        try:
            tf = self.tf_buffer.lookup_transform(
                target_frame, source_frame, rclpy.time.Time()
            )
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException) as ex:
            self.get_logger().warn(f"TF {source_frame} -> {target_frame} unavailable: {ex}")
            return None
        return do_transform_pose_stamped(self._marker_pose, tf)

    @staticmethod
    def _quat_to_axis(q, axis: str):
        """Return local `axis` ('x', 'y', or 'z') of orientation q, expressed
        in the parent frame, as an (x, y, z) unit vector."""
        n = (q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w) ** 0.5
        if n == 0.0:
            n = 1.0
        qx, qy, qz, qw = q.x / n, q.y / n, q.z / n, q.w / n
        if axis == 'x':
            return (
                1.0 - 2.0 * (qy * qy + qz * qz),
                2.0 * (qx * qy + qw * qz),
                2.0 * (qx * qz - qw * qy),
            )
        if axis == 'y':
            return (
                2.0 * (qx * qy - qw * qz),
                1.0 - 2.0 * (qx * qx + qz * qz),
                2.0 * (qy * qz + qw * qx),
            )
        if axis == 'z':
            return (
                2.0 * (qx * qz + qw * qy),
                2.0 * (qy * qz - qw * qx),
                1.0 - 2.0 * (qx * qx + qy * qy),
            )
        raise ValueError(f"Unknown axis '{axis}' (expected 'x', 'y', or 'z')")

    def _cube_pose_from_marker(self, marker_in_plan: PoseStamped) -> PoseStamped:
        """The cube isn't exactly at the tag: it sits `grasp_cube_distance`
        away from the tag along the tag's own local `grasp_cube_axis` axis.
        Same orientation as the marker (only position shifts)."""
        ax, ay, az = self._quat_to_axis(marker_in_plan.pose.orientation, self.grasp_cube_axis)
        p = marker_in_plan.pose.position
        cube = PoseStamped()
        cube.header = marker_in_plan.header
        cube.pose.position.x = p.x + self.grasp_cube_distance * ax
        cube.pose.position.y = p.y + self.grasp_cube_distance * ay
        cube.pose.position.z = p.z + self.grasp_cube_distance * az
        cube.pose.orientation = marker_in_plan.pose.orientation
        return cube

    def _offset_along_marker_z(self, marker_pose: PoseStamped, distance: float) -> Pose:
        """Build a target Pose offset from the marker along its Z axis.

        Gripper orientation = marker orientation (so gripper Z is aligned with
        tag Z). Position = marker_pos + approach_axis_sign * distance * marker_Z.
        """
        p = marker_pose.pose.position
        q = marker_pose.pose.orientation
        zx, zy, zz = self._quat_to_axis(q, 'z')

        n = (q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w) ** 0.5
        if n == 0.0:
            n = 1.0
        qx, qy, qz, qw = q.x / n, q.y / n, q.z / n, q.w / n

        d = self.approach_axis_sign * distance
        target = Pose()
        target.position.x = p.x + d * zx
        target.position.y = p.y + d * zy
        target.position.z = p.z + d * zz

        # Gripper orientation = marker orientation rotated 180° about its own Y
        # axis, so the gripper Z points into the tag (opposite the tag normal)
        # instead of away from it.
        rot_y180 = (0.0, 1.0, 0.0, 0.0)
        gx, gy, gz, gw = self._quat_mul((qx, qy, qz, qw), rot_y180)
        target.orientation.x = gx
        target.orientation.y = gy
        target.orientation.z = gz
        target.orientation.w = gw
        return target

    @staticmethod
    def _quat_mul(a, b):
        """Hamilton product a ⊗ b of two (x, y, z, w) quaternions."""
        ax, ay, az, aw = a
        bx, by, bz, bw = b
        return (
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        )

    @staticmethod
    def _lift_pose(base_pose: Pose, distance: float) -> Pose:
        """Vertical lift from base_pose along the WORLD/planning-frame Z axis
        (not the marker's local Z) — same orientation as base_pose."""
        p = Pose()
        p.position.x = base_pose.position.x
        p.position.y = base_pose.position.y
        p.position.z = base_pose.position.z + distance
        p.orientation = base_pose.orientation
        return p

    def _recompute_live_pose(self, distance: float) -> Pose:
        """Re-derive the `distance`-offset target from the *current* marker
        pose (as opposed to the frozen snapshot in self._run), so a moving
        marker can keep steering the reactive controller mid-motion. Returns
        None if the marker/TF isn't available right now."""
        if self._marker_pose is None:
            return None
        marker_in_plan = self._transform_marker_to(self.planning_frame)
        if marker_in_plan is None:
            return None
        cube_in_plan = self._cube_pose_from_marker(marker_in_plan)
        return self._offset_along_marker_z(cube_in_plan, distance)

    def _publish_target_tf(self):
        """Broadcast approach/grasp/lift_target as TF for RViz verification."""
        if self._marker_pose is None:
            return
        marker_in_plan = self._transform_marker_to(self.planning_frame)
        if marker_in_plan is None:
            return
        cube_in_plan = self._cube_pose_from_marker(marker_in_plan)
        approach_pose = self._offset_along_marker_z(cube_in_plan, self.approach_distance)
        grasp_pose = self._offset_along_marker_z(cube_in_plan, self.grasp_distance)
        lift_pose = self._lift_pose(grasp_pose, self.lift_distance)
        self._broadcast_pose(cube_in_plan.pose, 'cube_target')
        self._broadcast_pose(approach_pose, 'approach_target')
        self._broadcast_pose(grasp_pose, 'grasp_target')
        self._broadcast_pose(lift_pose, 'lift_target')

    def _broadcast_pose(self, pose: Pose, child_frame: str):
        """Broadcast a Pose (in the planning frame) as a TF frame."""
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.planning_frame
        tf.child_frame_id = child_frame
        tf.transform.translation.x = pose.position.x
        tf.transform.translation.y = pose.position.y
        tf.transform.translation.z = pose.position.z
        tf.transform.rotation = pose.orientation
        self.tf_broadcaster.sendTransform(tf)

    async def execute_trajectory(self, target_pose: Pose, live_pose_fn=None) -> bool:
        """Plan and execute a motion to target_pose (moves the arm).

        Stores the goal_handle so grasp_trigger_cb can cancel it if requested.

        If `live_pose_fn` is given, it's polled at mpc_goal_rate for the
        duration of the goal and its result republished on mpc_goal_topic —
        this lets the in-flight reactive controller keep following a marker
        that moves after the goal was sent, instead of only ever chasing the
        pose captured at trigger time.
        """
        if not self.execute_client.server_is_ready():
            self.get_logger().error("Execute trajectory action not available")
            return False

        goal = SendTrajectory.Goal()
        goal.target_pose = target_pose
        goal.allow_cached = False  # always plan+execute fresh for this target
        self.get_logger().info(
            f"Executing trajectory: target=({target_pose}"
        )
        self._on_target_reached = False
        goal_handle = await self.execute_client.send_goal_async(
            goal, feedback_callback=self._on_execute_feedback
        )
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Execute goal rejected")
            return False

        # Store goal_handle so grasp_trigger_cb (and the feedback callback,
        # once on_target=True) can cancel it.
        self._trajectory_goal_handle = goal_handle
        live_goal_timer = None
        if live_pose_fn is not None:
            live_goal_timer = self.create_timer(
                1.0 / self.mpc_goal_rate, lambda: self._publish_live_goal(live_pose_fn))
        try:
            result_response = await goal_handle.get_result_async()
            if self._on_target_reached:
                # Server is in continuous servoing (TRACKING) and never ends
                # the goal on its own — we cancel it ourselves as soon as
                # feedback reports on_target, and treat that as success.
                self.get_logger().info("On target: trajectory cancelled as success")
                return True
            result = result_response.result
            if result.success:
                self.get_logger().info(f"Trajectory succeeded: {result.message}")
                return True
            self.get_logger().error(f"Trajectory failed: {result.message}")
            return False
        finally:
            if live_goal_timer is not None:
                live_goal_timer.cancel()
                self.destroy_timer(live_goal_timer)
            self._trajectory_goal_handle = None

    def _publish_live_goal(self, live_pose_fn) -> None:
        """Timer callback: recompute the live target and publish it on
        mpc_goal_topic. Silently skips a tick if the marker/TF is momentarily
        unavailable rather than steering the controller with a stale pose."""
        pose = live_pose_fn()
        if pose is not None:
            self.mpc_goal_pub.publish(pose)

    def _on_execute_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f"  [{fb.state}] progress={fb.step_progression:.2f} "
            f"err={fb.position_error:.4f}m on_target={fb.on_target}",
            throttle_duration_sec=1.0,
        )
        if fb.on_target and not self._on_target_reached and self._trajectory_goal_handle is not None:
            self._on_target_reached = True
            self.get_logger().info("on_target=True, cancelling trajectory goal")
            self._trajectory_goal_handle.cancel_goal_async()

    async def call_gripper_action(self, position: float, grasp_object = False) -> bool:
        """Send a ParallelGripperCommand goal for gripper_joint_name to position.

        Note: the real Robotiq controller only exposes a `position` command
        interface (effort/velocity command interfaces were intentionally not
        claimed, since the hardware doesn't support them) — so this goal only
        ever sets position.
        """
        if self.simulate_gripper:
            action = "OPEN" if position <= self.gripper_open_position else "CLOSE"
            self.get_logger().info(f"[SIM] Gripper {action} (position={position:.3f})")
            return True

        if not self.gripper_client.server_is_ready():
            self.get_logger().error("Gripper action server not available")
            return False

        goal_msg = ParallelGripperCommand.Goal()
        goal_msg.command.name = [self.gripper_joint_name]
        goal_msg.command.position = [position]
        self.get_logger().info(f"Gripper command: position={position:.3f}")
        goal_handle = await self.gripper_client.send_goal_async(goal_msg)
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Gripper goal rejected")
            return False

        # Store goal_handle so grasp_trigger_cb can cancel if needed.
        self._gripper_goal_handle = goal_handle
        try:
            result = (await goal_handle.get_result_async()).result
            if(grasp_object):
                self.get_logger().info(f"Gripper done: Grasp object={result.stalled}")
                return result.stalled
            else:
                self.get_logger().info(f"Gripper done: reached_goal={result.reached_goal}")
                return result.reached_goal
        finally:
            self._gripper_goal_handle = None

    async def call_set_mask_service(self, pose: Pose) -> bool:
        """Exclude the target object (cuboid, mask_object_dimensions) from
        world collision at `pose`, via curobo_ros's set_mask service."""
        if not self.mask_client.service_is_ready():
            self.get_logger().error("set_mask service not available")
            return False

        req = SetMask.Request()
        req.type = SetMask.Request.CUBOID
        req.name = self.mask_object_name
        req.pose = pose
        req.dimensions = Vector3(
            x=self.mask_object_dimensions[0],
            y=self.mask_object_dimensions[1],
            z=self.mask_object_dimensions[2],
        )
        resp = await self.mask_client.call_async(req)
        self.get_logger().info(
            f"set_mask '{self.mask_object_name}': success={resp.success} msg=\"{resp.message}\"")
        return resp.success

    async def _call_service_with_timeout(self, client, request, label: str, timeout: float = None):
        """Call `request` on `client`, bounded by `timeout` (default
        doosan_service_timeout — pass move_line_timeout for blocking motion
        calls, which only reply once the robot physically stops).

        Uses a ROS timer + rclpy.task.Future (not asyncio.wait_for — same
        reasoning as _wait_for_grasp_contact) so a Doosan service that never
        replies to a rejected/invalid command (observed: SetDesiredForce
        after a MOTION_HOLD fault) can't hang the state machine forever.
        Returns the response, or None on unavailable/timeout (already logged).
        """
        if timeout is None:
            timeout = self.doosan_service_timeout
        if not client.service_is_ready():
            self.get_logger().error(f"{label} service not available")
            return None
        call_future = client.call_async(request)
        outcome = Future()

        def _on_call_done(fut):
            if not outcome.done():
                outcome.set_result(fut)

        def _on_timeout():
            if not outcome.done():
                outcome.set_result(None)

        call_future.add_done_callback(_on_call_done)
        timer = self.create_timer(
            timeout, _on_timeout, callback_group=self._poll_callback_group)
        try:
            resolved = await outcome
        finally:
            timer.cancel()
            self.destroy_timer(timer)

        if resolved is None:
            self.get_logger().error(
                f"{label}: timed out after {timeout:.1f}s waiting for response")
            return None
        return resolved.result()

    async def call_enable_streaming(self, enable: bool) -> bool:
        """Toggle dsr_controller2's RT command streaming. Must be disabled
        before, and re-enabled after, any compliance/force call: left on, it
        keeps streaming commands (observed rejection: internal event
        'eSpeedJ') that TASK_COMPLIANCE_CONTROL rejects, faulting the
        controller into MOTION_HOLD."""
        resp = await self._call_service_with_timeout(
            self.enable_streaming_client, SetBool.Request(data=enable), "enable_streaming")
        if resp is None:
            return False
        self.get_logger().info(
            f"enable_streaming({enable}): success={resp.success} message=\"{resp.message}\"")
        return resp.success

    async def call_task_compliance_ctrl(self, stx, ref: int, ramp_time: float) -> bool:
        """Enter Doosan task compliance control with target stiffness `stx`
        (6 values: 3 translational N/m + 3 rotational Nm/rad), ramped in over
        `ramp_time` seconds in reference frame `ref`."""
        req = TaskComplianceCtrl.Request()
        req.stx = stx
        req.ref = ref
        req.time = ramp_time
        resp = await self._call_service_with_timeout(
            self.task_compliance_client, req, "task_compliance_ctrl")
        if resp is None:
            return False
        self.get_logger().info(f"task_compliance_ctrl: success={resp.success} stx={stx}")
        return resp.success

    async def call_release_compliance_ctrl(self) -> bool:
        """Exit compliance control and restore normal operation mode."""
        resp = await self._call_service_with_timeout(
            self.release_compliance_client, ReleaseComplianceCtrl.Request(), "release_compliance_ctrl")
        if resp is None:
            return False
        self.get_logger().info(f"release_compliance_ctrl: success={resp.success}")
        return resp.success

    async def call_move_line_relative(self, pos, ref: int, vel, acc) -> bool:
        """Relative MoveLine by `pos` ([dx,dy,dz] mm + [drx,dry,drz] deg) in
        reference frame `ref`.

        sync_type=SYNC: the reply normally arrives only once the robot has
        physically stopped. An early success=True reply means the command was
        accepted and dropped, not that the move was fast — the known cause is
        commanding during a compliance stiffness ramp (compliance_settle_time).
        """
        req = MoveLine.Request()
        req.pos = list(pos)
        req.vel = list(vel)
        req.acc = list(acc)
        req.time = 0.0
        req.radius = 0.0
        req.ref = ref
        req.mode = MOVE_MODE_RELATIVE
        req.blend_type = 0
        req.sync_type = SYNC
        self.get_logger().info(f"move_line: relative pos={req} ref={ref}")
        resp = await self._call_service_with_timeout(
            self.move_line_client, req, "move_line", timeout=self.move_line_timeout)
        if resp is None:
            return False
        self.get_logger().info(f"move_line: success={resp.success}")
        return resp.success

    async def call_check_motion(self):
        """Current motion status (DR_STATE_IDLE/INIT/BUSY), or None on
        unavailable/timeout/failure."""
        resp = await self._call_service_with_timeout(
            self.check_motion_client, CheckMotion.Request(), "check_motion")
        if resp is None or not resp.success:
            return None
        return resp.status

    async def wait_motion_idle(self, timeout: float = None) -> bool:
        """Poll check_motion until the robot reports DR_STATE_IDLE.

        Safety net rather than the primary completion signal: MoveLine's
        sync_type=SYNC already blocks until the robot stops, so this normally
        returns on the first poll. It exists so nothing releases compliance or
        reports success while the arm is still moving. Returns False on
        timeout (motion still running) or if check_motion is unreachable.
        """
        if timeout is None:
            timeout = self.motion_idle_timeout
        deadline = self.get_clock().now() + Duration(seconds=timeout)
        # Give the driver one poll period to leave IDLE before believing an
        # IDLE reading: check_motion right after MoveLine returns can still
        # report the pre-motion state.
        await self._sleep(self.motion_poll_period)
        while self.get_clock().now() < deadline:
            status = await self.call_check_motion()
            if status is None:
                self.get_logger().error("check_motion unavailable, cannot confirm motion end")
                return False
            if status == DR_STATE_IDLE:
                return True
            await self._sleep(self.motion_poll_period)
        self.get_logger().error(
            f"Motion still running after {timeout:.1f}s (check_motion != IDLE)")
        return False

    async def _sleep(self, seconds: float) -> None:
        """Await `seconds` without blocking the executor — a one-shot ROS timer
        resolving a Future, on _poll_callback_group for the same
        deadlock-avoidance reason as _call_service_with_timeout."""
        done = Future()
        timer = self.create_timer(
            seconds, lambda: done.done() or done.set_result(True),
            callback_group=self._poll_callback_group)
        try:
            await done
        finally:
            timer.cancel()
            self.destroy_timer(timer)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectGrasper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
