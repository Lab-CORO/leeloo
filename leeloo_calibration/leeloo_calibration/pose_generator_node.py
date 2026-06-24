#!/usr/bin/env python3
"""
pose_generator_node.py
======================
ROS2 node — package: leeloo_calibration

Interactive calibration pose generator for Tsai-Lenz hand-eye calibration.

Workflow:
1. Node starts → robot enters teach mode
2. User moves robot until ArUco marker is centered in camera
3. User calls /set_reference_pose → node computes 15 candidates and starts session
4. For each candidate: node drives robot there, user calls /confirm_pose or /skip_pose
5. /confirm_pose: reads CURRENT joint angles, runs validity check, saves if valid
6. After all candidates: calibration_poses.yaml is written automatically

Usage:
    ros2 run leeloo_calibration pose_generator_node

    ros2 service call /set_reference_pose std_srvs/srv/Trigger {}
    # For each of the 15 candidates:
    ros2 service call /confirm_pose std_srvs/srv/Trigger {}
    # or if unreachable / marker not visible:
    ros2 service call /skip_pose std_srvs/srv/Trigger {}
"""

import enum
import math
import os
import threading
import time
from datetime import datetime
from itertools import zip_longest

import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from dsr_msgs2.srv import SetRobotMode, SetSafetyMode
from curobo_msgs.srv import TrajectoryGeneration, SetPlanner
from curobo_msgs.action import SendTrajectory


# Native DSR topic order [J1,J2,J4,J5,J3,J6] → natural order [J1,J2,J3,J4,J5,J6]
_CALIB_ORDER = [0, 1, 4, 2, 3, 5]

# Conservative operational limits (rad) indexed by calib_order position
_J_LIMITS = [
    (-3.14, 3.14),  # j1
    (-1.60, 1.70),  # j2
    (-2.10, 2.10),  # j3
    (-3.14, 3.14),  # j4
    (-2.10, 2.10),  # j5
    (-3.14, 3.14),  # j6
]

_REDUNDANCY_THRESHOLD = 0.5  # rad, Euclidean distance in 6D joint space

# Tier 1: wrist-only variations (Δj4, Δj5, Δj6) — arm stays put, marker likely visible
_WRIST_DELTAS = [
    (0,            0,           0),
    (0,            math.pi/4,  0),
    (0,           -math.pi/4,  0),
    (math.pi/2,   0,           0),
    (-math.pi/2,  0,           0),
    (math.pi/2,   math.pi/4,  0),
    (-math.pi/2, -math.pi/4,  0),
    (0,            0,          math.pi/2),
]

# Tier 2: arm position variations (Δj1, Δj2, j3_absolute) — covers workspace zones
_ARM_DELTAS = [
    (+1.5,  0.0, -1.0),
    (+1.5, -0.5, +1.0),
    (+1.5, +0.5, -1.0),
    (-1.5,  0.0, +1.0),
    (-1.5, -0.5, -1.0),
    (-1.5, +0.5, +1.0),
    ( 0.0, +0.7, +1.0),
]


class SetupState(enum.Enum):
    WAITING_FOR_SERVICES = 'waiting_for_services'
    SET_ROBOT_MODE       = 'set_robot_mode'
    SET_ROBOT_MODE_WAIT  = 'set_robot_mode_wait'
    SET_SAFETY_MODE      = 'set_safety_mode'
    SET_SAFETY_MODE_WAIT = 'set_safety_mode_wait'
    READY                = 'ready'


_SETUP_TIMEOUTS = {
    SetupState.SET_ROBOT_MODE_WAIT:  5,
    SetupState.SET_SAFETY_MODE_WAIT: 5,
}


class PoseGeneratorNode(Node):

    def __init__(self):
        super().__init__('pose_generator_node')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('robot_mode_service',    '/dsr01/system/set_robot_mode')
        self.declare_parameter('safety_mode_service',   '/dsr01/system/set_safety_mode')
        self.declare_parameter('joint_states_topic',    '/dsr01/joint_states')
        self.declare_parameter('traj_generate_service', '/unified_planner/generate_trajectory')
        self.declare_parameter('traj_execute_action',   '/unified_planner/execute_trajectory')
        self.declare_parameter('set_planner_service',   '/unified_planner/set_planner')
        self.declare_parameter('output_file',           self._default_output_path())

        self._output = self.get_parameter('output_file').value

        # ── Callback group (reentrant — services called from background thread)
        self._cbg = ReentrantCallbackGroup()

        # ── Clients ───────────────────────────────────────────────────────────
        self._robot_mode_client = self.create_client(
            SetRobotMode,
            self.get_parameter('robot_mode_service').value,
            callback_group=self._cbg)
        self._safety_mode_client = self.create_client(
            SetSafetyMode,
            self.get_parameter('safety_mode_service').value,
            callback_group=self._cbg)
        self._gen_client = self.create_client(
            TrajectoryGeneration,
            self.get_parameter('traj_generate_service').value,
            callback_group=self._cbg)
        self._planner_client = self.create_client(
            SetPlanner,
            self.get_parameter('set_planner_service').value,
            callback_group=self._cbg)
        self._exec_client = ActionClient(
            self, SendTrajectory,
            self.get_parameter('traj_execute_action').value,
            callback_group=self._cbg)

        # ── Joint states ──────────────────────────────────────────────────────
        self._last_js: JointState | None = None
        self._js_lock = threading.Lock()
        self.create_subscription(
            JointState,
            self.get_parameter('joint_states_topic').value,
            self._on_joint_states, 10)

        # ── Services ──────────────────────────────────────────────────────────
        self.create_service(
            Trigger, 'set_reference_pose',
            self._on_set_reference_pose, callback_group=self._cbg)
        self.create_service(
            Trigger, 'confirm_pose',
            self._on_confirm_pose, callback_group=self._cbg)
        self.create_service(
            Trigger, 'skip_pose',
            self._on_skip_pose, callback_group=self._cbg)

        # ── Session state ─────────────────────────────────────────────────────
        self._saved_poses: list[dict] = []
        self._session_running = False
        self._confirm_event   = threading.Event()
        self._confirm_result: bool | None = None
        self._confirm_snapshot: JointState | None = None

        # ── Setup state machine ───────────────────────────────────────────────
        self._setup_state    = SetupState.WAITING_FOR_SERVICES
        self._ticks_in_state = 0
        self._setup_future   = None
        self.create_timer(1.0, self._setup_tick)

        self.get_logger().info('[INIT] PoseGeneratorNode started.')

    # =========================================================================
    # Setup state machine — puts robot in teach mode at startup
    # =========================================================================

    def _setup_tick(self):
        if self._setup_state == SetupState.READY:
            return

        self._ticks_in_state += 1
        timeout = _SETUP_TIMEOUTS.get(self._setup_state)
        if timeout and self._ticks_in_state > timeout:
            self.get_logger().error(
                f'[setup] Timeout at {self._setup_state.value} — continuing.')
            if self._setup_state == SetupState.SET_ROBOT_MODE_WAIT:
                self._go_setup(SetupState.SET_SAFETY_MODE)
            else:
                self._go_setup(SetupState.READY)
                self._print_ready()
            return

        match self._setup_state:
            case SetupState.WAITING_FOR_SERVICES: self._s_waiting()
            case SetupState.SET_ROBOT_MODE:       self._s_set_robot_mode()
            case SetupState.SET_ROBOT_MODE_WAIT:  self._s_set_robot_mode_wait()
            case SetupState.SET_SAFETY_MODE:      self._s_set_safety_mode()
            case SetupState.SET_SAFETY_MODE_WAIT: self._s_set_safety_mode_wait()

    def _go_setup(self, state: SetupState):
        self._setup_state    = state
        self._ticks_in_state = 0

    def _s_waiting(self):
        missing = [
            name for client, name in [
                (self._robot_mode_client,  'set_robot_mode'),
                (self._safety_mode_client, 'set_safety_mode'),
            ]
            if not client.service_is_ready()
        ]
        if missing:
            self.get_logger().info(
                f'[setup] Waiting for: {", ".join(missing)}',
                throttle_duration_sec=5.0)
            return
        self.get_logger().info('[setup] DSR services ready.')
        self._go_setup(SetupState.SET_ROBOT_MODE)

    def _s_set_robot_mode(self):
        req            = SetRobotMode.Request()
        req.robot_mode = 0  # ROBOT_MODE_MANUAL
        self._setup_future = self._robot_mode_client.call_async(req)
        self.get_logger().info('[setup] Setting ROBOT_MODE_MANUAL (0)...')
        self._go_setup(SetupState.SET_ROBOT_MODE_WAIT)

    def _s_set_robot_mode_wait(self):
        if not self._setup_future.done():
            return
        self.get_logger().info(f'[setup] robot_mode response: {self._setup_future.result()}')
        self._go_setup(SetupState.SET_SAFETY_MODE)

    def _s_set_safety_mode(self):
        req              = SetSafetyMode.Request()
        req.safety_mode  = 2  # SAFETY_MODE_RECOVERY (freedrive/teach)
        req.safety_event = 1  # SAFETY_MODE_EVENT_MOVE
        self._setup_future = self._safety_mode_client.call_async(req)
        self.get_logger().info('[setup] Setting SAFETY_MODE_RECOVERY (2) — teach mode...')
        self._go_setup(SetupState.SET_SAFETY_MODE_WAIT)

    def _s_set_safety_mode_wait(self):
        if not self._setup_future.done():
            return
        self.get_logger().info(f'[setup] safety_mode response: {self._setup_future.result()}')
        self._go_setup(SetupState.READY)
        self._print_ready()

    def _print_ready(self):
        self.get_logger().info(
            '\n'
            '╔══════════════════════════════════════════════════════════╗\n'
            '║  Robot in TEACH MODE — move it freely.                   ║\n'
            '╠══════════════════════════════════════════════════════════╣\n'
            '║  1. Move robot until ArUco marker is centered in camera. ║\n'
            '║  2. ros2 service call /set_reference_pose \\              ║\n'
            '║         std_srvs/srv/Trigger {}                          ║\n'
            '╚══════════════════════════════════════════════════════════╝')

    # =========================================================================
    # Joint states subscription
    # =========================================================================

    def _on_joint_states(self, msg: JointState):
        with self._js_lock:
            self._last_js = msg

    # =========================================================================
    # Service: /set_reference_pose
    # =========================================================================

    def _on_set_reference_pose(self, _req, response):
        if self._setup_state != SetupState.READY:
            response.success = False
            response.message = f'Node not ready (state: {self._setup_state.value}).'
            return response

        with self._js_lock:
            js = self._last_js
        if js is None:
            response.success = False
            response.message = 'No joint state received yet.'
            return response

        if self._session_running:
            response.success = False
            response.message = 'A session is already in progress.'
            return response

        raw   = [round(p, 10) for p in js.position]
        calib = [raw[i] for i in _CALIB_ORDER]

        tier1, tier2 = self._generate_candidates(calib)
        candidates = [p for pair in zip_longest(tier1, tier2)
                      for p in pair if p is not None]

        if not candidates:
            response.success = False
            response.message = 'Could not generate valid candidates from this reference pose.'
            return response

        self.get_logger().info(
            f'[reference] Set: [{", ".join(f"{v:.3f}" for v in calib)}]\n'
            f'[reference] {len(tier1)} wrist + {len(tier2)} arm = '
            f'{len(candidates)} candidates.')

        self._session_running = True
        threading.Thread(
            target=self._run_session,
            args=(candidates,),
            daemon=True).start()

        response.success = True
        response.message = (
            f'Reference recorded. {len(candidates)} candidates queued. '
            f'Driving to candidate 1 now. '
            f'Use /confirm_pose or /skip_pose at each stop.')
        return response

    # =========================================================================
    # Service: /confirm_pose
    # =========================================================================

    def _on_confirm_pose(self, _req, response):
        if not self._session_running:
            response.success = False
            response.message = 'No session running.'
            return response

        with self._js_lock:
            self._confirm_snapshot = self._last_js
        self._confirm_result = True
        self._confirm_event.set()

        response.success = True
        response.message = 'Confirmed — checking validity...'
        return response

    # =========================================================================
    # Service: /skip_pose
    # =========================================================================

    def _on_skip_pose(self, _req, response):
        if not self._session_running:
            response.success = False
            response.message = 'No session running.'
            return response

        self._confirm_result = False
        self._confirm_event.set()

        response.success = True
        response.message = 'Candidate skipped.'
        return response

    # =========================================================================
    # Interactive session (background thread)
    # =========================================================================

    def _run_session(self, candidates: list[list[float]]):
        n = len(candidates)
        self.get_logger().info(f'[session] Starting — {n} candidates.')

        for i, candidate in enumerate(candidates):
            self.get_logger().info(
                f'\n[session] ═══ Candidate {i + 1}/{n} ═══\n'
                f'  Target: [{", ".join(f"{v:.3f}" for v in candidate)}]')

            self._drive_to_candidate(candidate, i + 1, n)
            self._restore_teach_mode()
            self._wait_for_valid_confirmation(i + 1, n)

        self._write_yaml()
        self._session_running = False
        self.get_logger().info(
            f'\n[session] ═══ COMPLETE ═══\n'
            f'  {len(self._saved_poses)}/{n} poses saved → {self._output}')

    def _drive_to_candidate(self, candidate: list[float], num: int, total: int) -> bool:
        # Switch to autonomous mode
        req_mode = SetRobotMode.Request()
        req_mode.robot_mode = 1  # ROBOT_MODE_AUTONOMOUS
        resp = self._wait_for_future(self._robot_mode_client.call_async(req_mode), 5.0)
        if resp is None or not resp.success:
            self.get_logger().warn('[drive] SetRobotMode(1) failed — trying anyway.')

        # Joint-space planner
        req_plan = SetPlanner.Request()
        req_plan.planner_type = 5
        self._wait_for_future(self._planner_client.call_async(req_plan), 5.0)

        # Plan trajectory
        self.get_logger().info(f'[drive {num}/{total}] Planning trajectory...')
        req_gen = TrajectoryGeneration.Request()
        req_gen.target_joint_positions = list(candidate)
        gen_resp = self._wait_for_future(self._gen_client.call_async(req_gen), 15.0)

        if gen_resp is None or not gen_resp.success or not gen_resp.trajectory:
            msg = gen_resp.message if gen_resp else 'timeout'
            self.get_logger().error(
                f'[drive {num}/{total}] Planning failed: {msg}\n'
                '  Robot stays at current position — confirm or skip.')
            return False

        self.get_logger().info(
            f'[drive {num}/{total}] Executing ({len(gen_resp.trajectory)} pts)...')

        # Wait for action server
        if not self._exec_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(f'[drive {num}/{total}] Action server not available.')
            return False

        gh_future = self._exec_client.send_goal_async(SendTrajectory.Goal())
        gh = self._wait_for_future(gh_future, 10.0)
        if gh is None or not gh.accepted:
            self.get_logger().error(f'[drive {num}/{total}] Goal rejected.')
            return False

        result = self._wait_for_future(gh.get_result_async(), 120.0)
        if result is None or not result.result.success:
            msg = result.result.message if result else 'timeout'
            self.get_logger().error(f'[drive {num}/{total}] Execution failed: {msg}')
            return False

        self.get_logger().info(f'[drive {num}/{total}] Arrived.')
        return True

    def _restore_teach_mode(self):
        req_mode = SetRobotMode.Request()
        req_mode.robot_mode = 0  # ROBOT_MODE_MANUAL
        self._wait_for_future(self._robot_mode_client.call_async(req_mode), 5.0)

        req_safe = SetSafetyMode.Request()
        req_safe.safety_mode  = 2  # SAFETY_MODE_RECOVERY (teach)
        req_safe.safety_event = 1  # SAFETY_MODE_EVENT_MOVE
        self._wait_for_future(self._safety_mode_client.call_async(req_safe), 5.0)

        self.get_logger().info('[drive] Teach mode restored — adjust robot if needed.')

    def _wait_for_valid_confirmation(self, num: int, total: int):
        """Prompt user and loop until a valid pose is confirmed or candidate is skipped."""
        while True:
            self.get_logger().info(
                f'\n[session] Candidate {num}/{total} — check marker visibility.\n'
                f'  Poses saved so far: {len(self._saved_poses)}\n'
                f'  Confirm: ros2 service call /confirm_pose std_srvs/srv/Trigger {{}}\n'
                f'  Skip:    ros2 service call /skip_pose std_srvs/srv/Trigger {{}}')

            self._confirm_event.clear()
            self._confirm_event.wait()

            if not self._confirm_result:
                self.get_logger().info(f'[session] Candidate {num} skipped.')
                return

            snapshot = self._confirm_snapshot
            if snapshot is None:
                self.get_logger().warn('[confirm] No joint state — try again.')
                continue

            raw   = [round(p, 10) for p in snapshot.position]
            calib = [raw[i] for i in _CALIB_ORDER]

            ok_lim, why_lim = self._check_limits(calib)
            if not ok_lim:
                self.get_logger().warn(
                    f'[confirm] {why_lim} — adjust robot and call /confirm_pose again.')
                continue

            is_red, why_red = self._is_redundant(calib)
            if is_red:
                self.get_logger().warn(
                    f'[confirm] {why_red} — adjust robot and call /confirm_pose again.')
                continue

            pose_num = len(self._saved_poses) + 1
            self._saved_poses.append({
                'index':       pose_num,
                'timestamp':   datetime.now().isoformat(timespec='seconds'),
                'joint_names': list(snapshot.name),
                'positions':   raw,
                'calib_order': calib,
            })
            self.get_logger().info(
                f'[session] Pose {pose_num} saved: '
                f'[{", ".join(f"{v:.3f}" for v in calib)}]')
            return

    # =========================================================================
    # Candidate generation
    # =========================================================================

    def _generate_candidates(
            self, ref: list[float]) -> tuple[list[list[float]], list[list[float]]]:
        accepted: list[list[float]] = []

        tier1: list[list[float]] = []
        for dj4, dj5, dj6 in _WRIST_DELTAS:
            pose = [
                ref[0],
                ref[1],
                ref[2],
                self._clip(ref[3] + dj4, 3),
                self._clip(ref[4] + dj5, 4),
                self._clip(ref[5] + dj6, 5),
            ]
            if not self._candidate_is_redundant(pose, accepted):
                accepted.append(pose)
                tier1.append(pose)

        tier2: list[list[float]] = []
        for dj1, dj2, j3_abs in _ARM_DELTAS:
            pose = [
                self._clip(ref[0] + dj1, 0),
                self._clip(ref[1] + dj2, 1),
                self._clip(j3_abs,       2),
                ref[3],
                ref[4],
                ref[5],
            ]
            if not self._candidate_is_redundant(pose, accepted):
                accepted.append(pose)
                tier2.append(pose)

        total = len(tier1) + len(tier2)
        if total < 15:
            self.get_logger().warn(
                f'[gen] Only {total} candidates generated (< 15). '
                'Consider a different reference pose.')

        return tier1, tier2

    def _clip(self, value: float, joint_idx: int) -> float:
        lo, hi = _J_LIMITS[joint_idx]
        return max(lo, min(hi, value))

    def _candidate_is_redundant(
            self, pose: list[float], accepted: list[list[float]]) -> bool:
        for existing in accepted:
            dist = math.sqrt(sum((a - b) ** 2 for a, b in zip(pose, existing)))
            if dist < _REDUNDANCY_THRESHOLD:
                return True
        return False

    # =========================================================================
    # Validity checks for confirmed poses
    # =========================================================================

    def _check_limits(self, calib: list[float]) -> tuple[bool, str]:
        for i, (v, (lo, hi)) in enumerate(zip(calib, _J_LIMITS)):
            if not lo <= v <= hi:
                return False, f'j{i + 1}={v:.3f} out of [{lo:.2f}, {hi:.2f}]'
        return True, ''

    def _is_redundant(self, calib: list[float]) -> tuple[bool, str]:
        for p in self._saved_poses:
            dist = math.sqrt(
                sum((a - b) ** 2 for a, b in zip(calib, p['calib_order'])))
            if dist < _REDUNDANCY_THRESHOLD:
                return True, (
                    f"Too close to pose {p['index']} "
                    f"(distance={dist:.3f} < {_REDUNDANCY_THRESHOLD} rad)")
        return False, ''

    # =========================================================================
    # Async helper — polls a future from the background thread
    # =========================================================================

    def _wait_for_future(self, future, timeout: float):
        start = time.monotonic()
        while not future.done():
            if time.monotonic() - start > timeout:
                self.get_logger().error(f'[wait] Timeout ({timeout:.0f}s).')
                return None
            time.sleep(0.05)
        return future.result()

    # =========================================================================
    # YAML output
    # =========================================================================

    def _write_yaml(self):
        data = {
            'metadata': {
                'saved_at':          datetime.now().isoformat(timespec='seconds'),
                'total_poses':       len(self._saved_poses),
                'joint_order_raw':   'J1, J2, J4, J5, J3, J6  (native /dsr01/joint_states order)',
                'joint_order_calib': 'J1, J2, J3, J4, J5, J6  (natural order, used for planning)',
            },
            'poses':                   self._saved_poses,
            'calibration_poses_ready': [p['calib_order'] for p in self._saved_poses],
        }
        os.makedirs(os.path.dirname(os.path.abspath(self._output)), exist_ok=True)
        try:
            with open(self._output, 'w') as f:
                yaml.dump(data, f, default_flow_style=None,
                          allow_unicode=True, sort_keys=False)
        except OSError as e:
            self.get_logger().error(f'[yaml] Write failed: {e}')

    @staticmethod
    def _default_output_path() -> str:
        return os.path.normpath(os.path.join(
            os.path.dirname(__file__), '..', 'config', 'calibration_poses.yaml'))


# =============================================================================
# main
# =============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = PoseGeneratorNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
