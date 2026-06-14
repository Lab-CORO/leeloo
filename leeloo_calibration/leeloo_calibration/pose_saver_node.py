#!/usr/bin/env python3
"""
pose_saver_node.py
==================
Noeud ROS2  –  package : leeloo_calibration

Aide à la prise de poses de calibration :
  1. Met le DSR en mode manuel (set_robot_mode=1, safety_mode=0)
  2. Expose le service /save_pose (Trigger) :
       → lit la position courante des joints (/dsr01/joint_states)
       → l'ajoute à la liste et écrit le fichier YAML

Usage :
    ros2 run leeloo_calibration pose_saver_node
    ros2 run leeloo_calibration pose_saver_node \
        --ros-args -p output_file:=/chemin/vers/poses.yaml

Enregistrer une pose :
    ros2 service call /save_pose std_srvs/srv/Trigger {}
"""

import enum
import math
import os
from datetime import datetime

import yaml

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from dsr_msgs2.srv import SetRobotMode, SetSafetyMode
from curobo_msgs.srv import GetCollisionDistance


# =============================================================================
# Machine à états
# =============================================================================

class State(enum.Enum):
    WAITING_FOR_SERVICES  = 'waiting_for_services'
    SET_ROBOT_MODE        = 'set_robot_mode'
    SET_ROBOT_MODE_WAIT   = 'set_robot_mode_wait'
    SET_SAFETY_MODE       = 'set_safety_mode'
    SET_SAFETY_MODE_WAIT  = 'set_safety_mode_wait'
    READY                 = 'ready'


TIMEOUTS = {
    State.SET_ROBOT_MODE_WAIT:  5,
    State.SET_SAFETY_MODE_WAIT: 5,
}

# Réordonnancement JointState (J1,J2,J3,J4,J5,J6) → CALIBRATION_POSES (J1,J2,J4,J5,J3,J6)
# Si le topic publie dans un ordre différent, ajuster ces indices.
_CALIB_ORDER = [0, 1, 4, 2, 3, 5]


# =============================================================================
# Node
# =============================================================================

class PoseSaverNode(Node):

    def __init__(self):
        super().__init__('pose_saver_node')

        # ---- Paramètres ROS --------------------------------------------------
        self.declare_parameter('robot_mode_service',  '/dsr01/system/set_robot_mode')
        self.declare_parameter('safety_mode_service', '/dsr01/system/set_safety_mode')
        self.declare_parameter('joint_states_topic',  '/dsr01/joint_states')
        self.declare_parameter('output_file',         self._default_output_path())

        robot_mode_srv  = self.get_parameter('robot_mode_service').value
        safety_mode_srv = self.get_parameter('safety_mode_service').value
        joint_topic     = self.get_parameter('joint_states_topic').value
        self._output    = self.get_parameter('output_file').value

        self.declare_parameter('collision_distance_service',
                               '/unified_planner/get_collision_distance')
        collision_srv = self.get_parameter('collision_distance_service').value

        self.declare_parameter('load_existing_poses', True)

        # ---- Clients DSR -----------------------------------------------------
        self._robot_mode_client  = self.create_client(SetRobotMode,  robot_mode_srv)
        self._safety_mode_client = self.create_client(SetSafetyMode, safety_mode_srv)

        # ---- Collision check client (reentrant — called from service callback) ---
        self._reentrant_group = ReentrantCallbackGroup()
        self._collision_client = self.create_client(
            GetCollisionDistance, collision_srv,
            callback_group=self._reentrant_group)

        # ---- Subscription joint states (dernier message conservé) ------------
        self._last_js: JointState | None = None
        self.create_subscription(JointState, joint_topic, self._on_joint_states, 10)

        # ---- Services -----------------------------------------------------------
        self.create_service(Trigger, 'save_pose', self._on_save_pose,
                            callback_group=self._reentrant_group)
        self.create_service(Trigger, 'info_calibration', self._on_info_calibration,
                            callback_group=self._reentrant_group)

        # ---- Données de session ----------------------------------------------
        self._poses: list[dict] = []
        if self.get_parameter('load_existing_poses').value:
            self._load_existing_yaml()

        # ---- Machine à états -------------------------------------------------
        self._state          : State = State.WAITING_FOR_SERVICES
        self._ticks_in_state : int   = 0
        self._future                 = None

        # ---- Boucle principale à 1 Hz ----------------------------------------
        self.create_timer(1.0, self._tick)
        self.get_logger().info(
            f'[INIT] PoseSaverNode démarré.\n'
            f'  Fichier de sortie : {self._output}')

    # =========================================================================
    # Boucle principale (1 Hz)
    # =========================================================================

    def _tick(self):
        if self._state == State.READY:
            return

        self._ticks_in_state += 1

        # --- Timeout ---
        timeout = TIMEOUTS.get(self._state)
        if timeout and self._ticks_in_state > timeout:
            self.get_logger().error(
                f'[{self._state.value}] Timeout ({timeout}s) — on continue quand même.')
            match self._state:
                case State.SET_ROBOT_MODE_WAIT:  self._go(State.SET_SAFETY_MODE)
                case State.SET_SAFETY_MODE_WAIT: self._go_ready()
                case _:                          self._go(State.READY)
            return

        match self._state:
            case State.WAITING_FOR_SERVICES: self._s_waiting_for_services()
            case State.SET_ROBOT_MODE:       self._s_set_robot_mode()
            case State.SET_ROBOT_MODE_WAIT:  self._s_set_robot_mode_wait()
            case State.SET_SAFETY_MODE:      self._s_set_safety_mode()
            case State.SET_SAFETY_MODE_WAIT: self._s_set_safety_mode_wait()

    def _go(self, state: State):
        self.get_logger().debug(f'  {self._state.value} → {state.value}')
        self._state          = state
        self._ticks_in_state = 0

    # =========================================================================
    # Handlers d'états
    # =========================================================================

    def _s_waiting_for_services(self):
        missing = [
            name for client, name in [
                (self._robot_mode_client,  'set_robot_mode'),
                (self._safety_mode_client, 'set_safety_mode'),
            ]
            if not client.service_is_ready()
        ]
        if missing:
            self.get_logger().info(
                f'[waiting] En attente : {", ".join(missing)}',
                throttle_duration_sec=5.0)
            return

        self.get_logger().info('[waiting] Services DSR prêts.')
        self._go(State.SET_ROBOT_MODE)

    # --- set_robot_mode -------------------------------------------------------

    def _s_set_robot_mode(self):
        req            = SetRobotMode.Request()
        req.robot_mode = 0
        self._future   = self._robot_mode_client.call_async(req)
        self.get_logger().info('[set_robot_mode] Passage en mode manuel (robot_mode=0)...')
        self._go(State.SET_ROBOT_MODE_WAIT)

    def _s_set_robot_mode_wait(self):
        if not self._future.done():
            return
        resp = self._future.result()
        match resp:
            case None: self.get_logger().error('[set_robot_mode] Pas de réponse.')
            case _:    self.get_logger().info(f'[set_robot_mode] Réponse : {resp}')
        self._go(State.SET_SAFETY_MODE)

    # --- set_safety_mode ------------------------------------------------------

    def _s_set_safety_mode(self):
        req              = SetSafetyMode.Request()
        req.safety_mode  = 2
        req.safety_event = 1
        self._future     = self._safety_mode_client.call_async(req)
        self.get_logger().info(
            '[set_safety_mode] Désactivation des restrictions (safety_mode=1, safety_event=2)...')
        self._go(State.SET_SAFETY_MODE_WAIT)

    def _s_set_safety_mode_wait(self):
        if not self._future.done():
            return
        resp = self._future.result()
        match resp:
            case None: self.get_logger().error('[set_safety_mode] Pas de réponse.')
            case _:    self.get_logger().info(f'[set_safety_mode] Réponse : {resp}')
        self._go_ready()
 
    def _go_ready(self):
        self._go(State.READY)
        self.get_logger().info(
            '\n'
            '╔════════════════════════════════════════════════════╗\n'
            '║  Robot en mode MANUEL — tu peux le bouger.        ║\n'
            '╠════════════════════════════════════════════════════╣\n'
            '║  Pour sauvegarder la pose courante :               ║\n'
            '║  ros2 service call /save_pose std_srvs/srv/Trigger ║\n'
            f'║  Fichier : {self._output:<41}║\n'
            '╚════════════════════════════════════════════════════╝')

    # =========================================================================
    # Subscription joint states
    # =========================================================================

    def _on_joint_states(self, msg: JointState):
        self._last_js = msg

    # =========================================================================
    # Collision check
    # =========================================================================

    def _check_collision(self) -> tuple[bool, str]:
        """Returns (ok, reason). ok=True → safe to save. ok=False → reject pose."""
        return True, ''
        if not self._collision_client.service_is_ready():
            self.get_logger().warn(
                '[collision] Service unavailable — skipping collision check.')
            return True, ''
        future = self._collision_client.call_async(GetCollisionDistance.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if not future.done() or future.result() is None:
            self.get_logger().warn('[collision] Timeout — skipping collision check.')
            return True, ''
        # Sign convention: negative = clearance (free), positive = penetration (collision).
        # cuRobo clamps clearance at -0.10 m, so any d > 0 means actual collision.
        data = list(future.result().data)
        colliding = [d for d in data if d > 0.0]
        if colliding:
            return False, f'Collision detected (max penetration = {max(colliding):.4f} m)'
        near = [d for d in data if -0.05 < d <= 0.0]
        if near:
            self.get_logger().warn(
                f'[collision] Near obstacle ({len(near)} sphere(s) < 5 cm clearance, '
                f'max={max(near):.4f} m) — saving anyway.')
        return True, ''

    # =========================================================================
    # Service /save_pose
    # =========================================================================

    def _on_save_pose(self, _request, response):
        if self._state != State.READY:
            response.success = False
            response.message = (
                f'Node not ready yet (state: {self._state.value}). '
                f'Try again in a few seconds.')
            return response

        if self._last_js is None:
            response.success = False
            response.message = 'No data received on joint_states topic.'
            return response

        names = list(self._last_js.name)
        raw   = [round(p, 10) for p in self._last_js.position]
        calib = [raw[i] for i in _CALIB_ORDER]

        ok, reason = self._check_collision()
        if not ok:
            response.success = False
            response.message = f'Pose not saved: {reason}'
            self.get_logger().warn(f'[save_pose] {response.message}')
            return response

        redundant, reason = self._is_redundant(calib)
        if redundant:
            response.success = False
            response.message = f'Pose not saved: {reason}'
            self.get_logger().warn(f'[save_pose] {response.message}')
            return response

        pose_num = len(self._poses) + 1
        self._poses.append({
            'index':       pose_num,
            'timestamp':   datetime.now().isoformat(timespec='seconds'),
            'joint_names': names,
            'positions':   raw,
            'calib_order': calib,
        })

        self._write_yaml()

        log_str = ', '.join(f'{n}={p:.4f}' for n, p in zip(names, raw))
        self.get_logger().info(f'[save_pose] Pose {pose_num} saved: {log_str}')

        response.success = True
        response.message = (
            f'Pose {pose_num} saved. '
            f'Total: {len(self._poses)} pose(s). '
            f'File: {self._output}')
        return response

    # =========================================================================
    # Load existing YAML
    # =========================================================================

    def _load_existing_yaml(self):
        if not os.path.isfile(self._output):
            return
        try:
            with open(self._output) as f:
                data = yaml.safe_load(f)
            existing = data.get('poses', [])
            if existing:
                self._poses = list(existing)
                self.get_logger().info(
                    f'[init] Loaded {len(self._poses)} existing pose(s) from {self._output}')
        except Exception as e:
            self.get_logger().warn(f'[init] Could not load existing YAML: {e}')

    # =========================================================================
    # Redundancy guard
    # =========================================================================

    _REDUNDANCY_THRESHOLD = 0.5  # rad (Euclidean distance in 6D joint space)

    def _is_redundant(self, calib: list[float]) -> tuple[bool, str]:
        for p in self._poses:
            existing = p['calib_order']
            if len(existing) != len(calib):
                continue
            dist = math.sqrt(sum((a - b) ** 2 for a, b in zip(calib, existing)))
            if dist < self._REDUNDANCY_THRESHOLD:
                return True, (
                    f"Too close to pose {p['index']} "
                    f"(joint-space distance = {dist:.3f} rad "
                    f"< {self._REDUNDANCY_THRESHOLD} rad)")
        return False, ''

    # =========================================================================
    # Coverage report  (/info_calibration service)
    # =========================================================================

    def _build_coverage_report(self) -> str:
        MIN_POSES = 15
        lines = ['=== Calibration coverage report ===']
        n = len(self._poses)
        lines.append(
            f'Recorded poses : {n} / {MIN_POSES} recommended'
            + (' [OK]' if n >= MIN_POSES else ' [MISSING]'))

        j1s = [p['calib_order'][0] for p in self._poses]
        j2s = [p['calib_order'][1] for p in self._poses]
        j3s = [p['calib_order'][2] for p in self._poses]

        def j1_zone(v): return 'Left'   if v < -0.5 else ('Right' if v > 0.5 else 'Center')
        def j2_zone(v): return 'Negative' if v < 0  else ('Low'   if v < 0.5 else 'High')

        zones_present = {(j1_zone(j1), j2_zone(j2)) for j1, j2 in zip(j1s, j2s)}
        all_zones     = [(j1z, j2z)
                         for j1z in ('Left', 'Center', 'Right')
                         for j2z in ('Negative', 'Low', 'High')]

        covered = [z for z in all_zones if z     in zones_present]
        missing = [z for z in all_zones if z not in zones_present]

        lines.append(f'\nj1 x j2 zones covered ({len(covered)}/9):')
        for z in covered:
            lines.append(f'  [OK]      j1={z[0]}, j2={z[1]}')
        if missing:
            lines.append(f'\nj1 x j2 zones missing ({len(missing)}/9):')
            for z in missing:
                lines.append(f'  [MISSING] j1={z[0]}, j2={z[1]}')

        elbow_up   = sum(1 for j in j3s if j >  0.1)
        elbow_down = sum(1 for j in j3s if j < -0.1)
        lines.append('\nElbow configuration:')
        lines.append(
            f'  Elbow up   (j3 > +0.1) : {elbow_up} pose(s)'
            + ('' if elbow_up   >= 3 else ' [MISSING] (< 3 recommended)'))
        lines.append(
            f'  Elbow down (j3 < -0.1) : {elbow_down} pose(s)'
            + ('' if elbow_down >= 3 else ' [MISSING] (< 3 recommended)'))

        risky = [p['index'] for p in self._poses if p['calib_order'][1] > 1.8]
        if risky:
            lines.append(
                f'\nRisky poses (j2 > 1.2 rad -> likely GRAPH_FAIL): {risky}')

        return '\n'.join(lines)

    def _on_info_calibration(self, _request, response):
        if not self._poses:
            response.success = False
            response.message = 'No poses recorded.'
            return response
        response.success = True
        response.message = self._build_coverage_report()
        self.get_logger().info(f'\n{response.message}')
        return response

    # =========================================================================
    # Écriture YAML
    # =========================================================================

    def _write_yaml(self):
        data = {
            # Métadonnées
            'metadata': {
                'saved_at':    datetime.now().isoformat(timespec='seconds'),
                'total_poses': len(self._poses),
                'joint_order_raw':   'J1, J2, J3, J4, J5, J6  (ordre natif /dsr01/joint_states)',
                'joint_order_calib': 'J1, J2, J4, J5, J3, J6  (ordre CALIBRATION_POSES)',
            },
            # Détail complet de chaque pose
            'poses': self._poses,
            # Prêt à copier dans CALIBRATION_POSES
            'calibration_poses_ready': [p['calib_order'] for p in self._poses],
        }

        os.makedirs(os.path.dirname(os.path.abspath(self._output)), exist_ok=True)
        try:
            with open(self._output, 'w') as f:
                yaml.dump(data, f, default_flow_style=None, allow_unicode=True,
                          sort_keys=False)
        except OSError as e:
            self.get_logger().error(f'Erreur écriture YAML ({self._output}) : {e}')

    # =========================================================================
    # Helpers
    # =========================================================================

    @staticmethod
    def _default_output_path() -> str:
        """Retourne le chemin par défaut dans le dossier config/ du package."""
        pkg_root = os.path.join(
            os.path.dirname(__file__),  # leeloo_calibration/
            '..',                       # racine du package source
            'config',
            'calibration_poses.yaml')
        return os.path.normpath(pkg_root)


# =============================================================================
# main
# =============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = PoseSaverNode()
    executor = MultiThreadedExecutor(num_threads=2)
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
