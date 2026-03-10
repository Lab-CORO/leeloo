#!/usr/bin/env python3
"""
auto_calibration_server_v2.py
=============================
Noeud ROS2  –  package : leeloo_calibration

Action server RunCalibration (leeloo_msgs/action/RunCalibration).
La calibration démarre uniquement sur réception d'un goal.

L'execute_callback pilote directement la procédure (pas de timer externe).
Les appels de services sont non-bloquants (call_async) et attendus par
polling (_wait_future), ce qui permet au spin de continuer à tourner.

Usage :
    ros2 run leeloo_calibration auto_calibration_server_v2
"""

import os
import time

import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from ros2_markertracker_interfaces.srv import CapturePoint
from curobo_msgs.srv import TrajectoryGeneration, SetPlanner
from curobo_msgs.action import SendTrajectory
from leeloo_msgs.action import RunCalibration

_DEFAULT_POSES_FILE = '/home/ros2_ws/src/leeloo_calibration/config/calibration_poses.yaml'
_DEFAULT_WAIT_S     = 5.0


class AutoCalibrationNode(Node):

    def __init__(self):
        super().__init__('auto_calibration_node')

        # ---- Paramètres ROS --------------------------------------------------
        self.declare_parameter('capture_point_service', '/hand_eye_calibration/capture_point')
        self.declare_parameter('traj_generate_service', '/unified_planner/generate_trajectory')
        self.declare_parameter('traj_execute_action',   '/unified_planner/execute_trajectory')
        self.declare_parameter('set_planner_service',   '/unified_planner/set_planner')
        self.declare_parameter('calibration_poses_file', _DEFAULT_POSES_FILE)
        self.declare_parameter('calibration_result_file',
            '/home/ros2_ws/src/tool_box/camera_calibration/config/kinect_hand_eye_result.yaml')

        # ---- Callback group pour les clients ---------------------------------
        # ReentrantCallbackGroup requis : l'execute_callback de l'action server
        # tourne dans son propre thread et doit pouvoir recevoir les réponses
        # de services en parallèle (sinon deadlock avec MutuallyExclusiveCallbackGroup).
        self._clients_cb_group = ReentrantCallbackGroup()

        # ---- Clients ---------------------------------------------------------
        self._gen_client     = self.create_client(
            TrajectoryGeneration,
            self.get_parameter('traj_generate_service').value,
            callback_group=self._clients_cb_group)
        self._planner_client = self.create_client(
            SetPlanner,
            self.get_parameter('set_planner_service').value,
            callback_group=self._clients_cb_group)
        self._exec_client    = ActionClient(
            self, SendTrajectory,
            self.get_parameter('traj_execute_action').value,
            callback_group=self._clients_cb_group)
        self._capture_client = self.create_client(
            CapturePoint,
            self.get_parameter('capture_point_service').value,
            callback_group=self._clients_cb_group)

        # ---- Action server ---------------------------------------------------
        self._action_server = ActionServer(
            self,
            RunCalibration,
            'run_calibration',
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
        )

        self._running = False
        self.get_logger().info("[INIT] AutoCalibrationNode démarré. En attente d'un goal RunCalibration.")

    # =========================================================================
    # Callbacks de l'action server
    # =========================================================================

    def _goal_cb(self, goal_request):
        """Accepte le goal seulement si aucune calibration n'est en cours."""
        if self._running:
            self.get_logger().warn('[action] Goal refusé : calibration déjà en cours.')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        """Accepte toutes les demandes d'annulation."""
        self.get_logger().info('[action] Annulation demandée.')
        return CancelResponse.ACCEPT

    def _execute_cb(self, goal_handle):
        """
        Pilote la procédure complète de calibration.
        Appelé dans un thread dédié par l'action server — peut bloquer.
        """
        self._running = True

        n    = goal_handle.request.num_poses
        wait = goal_handle.request.wait_between_poses_s or _DEFAULT_WAIT_S
        n_eff = n if n > 0 else -1

        self.get_logger().info(
            f'[action] Goal accepté — num_poses={n}, wait={wait}s')

        result                 = RunCalibration.Result()
        result.poses_completed = 0
        result.poses_failed    = 0

        # ---- 1. Attente des serveurs -----------------------------------------
        self.get_logger().info('[action] Attente des serveurs...')
        while True:
            missing = self._check_servers()
            if not missing:
                break
            fb = RunCalibration.Feedback()
            fb.current_pose     = 0
            fb.total_poses      = 0
            fb.status           = 'Waiting'
            fb.percent_complete = 0.0
            goal_handle.publish_feedback(fb)
            self.get_logger().info(
                f'  En attente : {", ".join(missing)}',
                throttle_duration_sec=5.0)
            if not self._sleep_or_cancel(1.0, goal_handle):
                result.success = False
                result.message = "Annulé pendant l'attente des serveurs."
                goal_handle.canceled()
                self._running = False
                return result

        self.get_logger().info('[action] Tous les serveurs sont prêts.')

        # ---- 2. Chargement des poses -----------------------------------------
        poses_file = self.get_parameter('calibration_poses_file').value
        poses, error = self._load_poses(poses_file, n_eff)
        if error:
            self.get_logger().fatal(error)
            result.success = False
            result.message = error
            goal_handle.abort()
            self._running = False
            return result

        total = len(poses)
        self.get_logger().info(
            f'[action] {total} poses chargées — démarrage de la calibration.')

        # ---- 3. Activation du planner (type joint-space) --------------------
        req              = SetPlanner.Request()
        req.planner_type = 5
        resp = self._wait_future(
            self._planner_client.call_async(req),
            timeout=5.0, goal_handle=goal_handle)
        if resp is not None and resp.success:
            self.get_logger().info('[action] Planner joint-space (type=5) activé.')
        else:
            self.get_logger().warn('[action] SetPlanner sans réponse valide. On continue.')

        # ---- 4. Boucle principale sur les poses -----------------------------
        completed = 0
        failed    = 0
        last_calibration_transform = None

        for i, pose in enumerate(poses):

            # Vérification annulation en début de pose
            if goal_handle.is_cancel_requested:
                result.success         = False
                result.poses_completed = completed
                result.poses_failed    = failed
                result.message         = f'Annulé à la pose {i + 1}/{total}.'
                goal_handle.canceled()
                self._running = False
                return result

            j1, j2, j3, j4, j5, j6 = pose[0], pose[1], pose[4], pose[2], pose[3], pose[5]
            self.get_logger().info(
                f'--- Pose {i + 1}/{total} '
                f'[j1={j1:.2f} j2={j2:.2f} j3={j3:.2f} '
                f'j4={j4:.2f} j5={j5:.2f} j6={j6:.2f}] rad ---')

            def _fb(status):
                fb                  = RunCalibration.Feedback()
                fb.current_pose     = i + 1
                fb.total_poses      = total
                fb.status           = status
                fb.percent_complete = float(i) / total * 100.0
                goal_handle.publish_feedback(fb)

            # -- Génération de trajectoire -------------------------------------
            _fb('Generating')
            req_gen                        = TrajectoryGeneration.Request()
            req_gen.target_joint_positions = list(pose)
            gen_resp = self._wait_future(
                self._gen_client.call_async(req_gen),
                timeout=15.0, goal_handle=goal_handle)

            if gen_resp is None:
                self.get_logger().error(f'  Pose {i + 1} ignorée : generate timeout ou annulé.')
                failed += 1
                continue
            if not gen_resp.success or not gen_resp.trajectory:
                self.get_logger().error(
                    f'  Pose {i + 1} ignorée : generate échoué — {gen_resp.message}')
                failed += 1
                continue
            self.get_logger().info(
                f'  [generate] {len(gen_resp.trajectory)} points, dt={gen_resp.dt:.4f}s')

            # -- Envoi du goal d'exécution -------------------------------------
            _fb('Executing')
            traj_gh_future = self._exec_client.send_goal_async(
                SendTrajectory.Goal(),
                feedback_callback=self._traj_feedback_cb)
            traj_gh = self._wait_future(traj_gh_future, timeout=10.0, goal_handle=goal_handle)

            if traj_gh is None or not traj_gh.accepted:
                self.get_logger().error(f'  Pose {i + 1} ignorée : goal execute refusé.')
                failed += 1
                continue

            traj_res = self._wait_future(
                traj_gh.get_result_async(),
                timeout=120.0, goal_handle=goal_handle)

            if traj_res is None or not traj_res.result.success:
                msg = traj_res.result.message if traj_res else 'timeout'
                self.get_logger().error(
                    f'  Pose {i + 1} ignorée : execute échoué — {msg}')
                failed += 1
                continue
            self.get_logger().info('  [execute] Trajectoire exécutée avec succès.')

            # -- Capture -------------------------------------------------------
            _fb('Capturing')
            cap_resp = self._wait_future(
                self._capture_client.call_async(CapturePoint.Request()),
                timeout=10.0, goal_handle=goal_handle)

            if cap_resp is None:
                self.get_logger().error(f'  [capture] Pose {i + 1} : timeout ou annulé.')
            elif cap_resp.success:
                self.get_logger().info(f'  [capture] Sample capturé (pose {i + 1}).')
                t = cap_resp.transform
                if any([t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]):
                    last_calibration_transform = t
            else:
                self.get_logger().warn(f'  [capture] Pose {i + 1} : {cap_resp.message}')

            completed += 1
            self.get_logger().info(
                f'  Pose {i + 1}/{total} OK ✓  ({completed} OK, {failed} KO)')

            # -- Attente entre les poses ---------------------------------------
            if i < total - 1:
                if not self._sleep_or_cancel(wait, goal_handle):
                    result.success         = False
                    result.poses_completed = completed
                    result.poses_failed    = failed
                    result.message         = f'Annulé après {completed} poses capturées.'
                    goal_handle.canceled()
                    self._running = False
                    return result

        # ---- 5. Fin normale --------------------------------------------------
        self.get_logger().info(
            f'=== Calibration terminée : {completed}/{total} OK'
            + (f', {failed} échouées.' if failed else ' — aucune erreur.') + ' ===')

        # ---- 5b. Sauvegarde du résultat de calibration ----------------------
        if last_calibration_transform is not None:
            result_file = self.get_parameter('calibration_result_file').value
            self._save_calibration_yaml(last_calibration_transform, result_file)
        else:
            self.get_logger().warn(
                '[calibration] Pas assez de samples pour calculer un résultat (minimum 4).')

        result.success         = True
        result.poses_completed = completed
        result.poses_failed    = failed
        result.message         = (
            f'Calibration terminée : {completed}/{total} OK'
            + (f', {failed} échouées.' if failed else '.')
        )
        goal_handle.succeed()
        self._running = False
        return result

    # =========================================================================
    # Helpers
    # =========================================================================

    def _check_servers(self) -> list:
        """Retourne la liste des serveurs non encore disponibles."""
        missing = [
            name for client, name in [
                (self._gen_client,     'generate_trajectory'),
                (self._planner_client, 'set_planner'),
                (self._capture_client, 'capture_point'),
            ]
            if not client.service_is_ready()
        ]
        if not self._exec_client.server_is_ready():
            missing.append('execute_trajectory')
        return missing

    def _load_poses(self, poses_file: str, n: int):
        """
        Charge les poses depuis le fichier YAML.
        Retourne (poses, None) ou (None, message_erreur).
        """
        if not os.path.isfile(poses_file):
            return None, f'Fichier de poses introuvable : {poses_file}'
        with open(poses_file, 'r') as f:
            data = yaml.safe_load(f)
        # PATCH TEMPORAIRE : calib_order du YAML est faux (double inversion dans pose_saver_node —
        # _CALIB_ORDER conçu pour input [J1,J2,J3,J4,J5,J6] mais le topic DSR publie
        # [J1,J2,J4,J5,J3,J6]). Le champ 'positions' est lui directement du topic, donc correct.
        poses_entries = data.get('poses', [])
        if not poses_entries:
            return None, f'Clé "poses" vide ou absente dans {poses_file}'
        all_poses = [entry['calib_order'] for entry in poses_entries]
        poses = list(all_poses[:n] if 0 < n <= len(all_poses) else all_poses)
        return poses, None

    def _wait_future(self, future, timeout: float, goal_handle=None):
        """
        Attend la résolution d'un future par polling.
        Retourne le résultat, ou None si annulé ou timeout.
        """
        elapsed = 0.0
        step    = 0.1
        while not future.done():
            if goal_handle is not None and goal_handle.is_cancel_requested:
                return None
            time.sleep(step)
            elapsed += step
            if elapsed > timeout:
                self.get_logger().error(f'Timeout {timeout}s atteint.')
                return None
        return future.result()

    def _sleep_or_cancel(self, seconds: float, goal_handle) -> bool:
        """
        Attend `seconds` secondes avec vérification d'annulation toutes les 100 ms.
        Retourne False si annulé.
        """
        steps = max(1, int(seconds / 0.1))
        for i in range(steps):
            if goal_handle.is_cancel_requested:
                return False
            remaining = seconds - i * 0.1
            if i % 10 == 0:
                self.get_logger().info(f'  [wait] Prochaine pose dans {remaining:.0f}s...')
            time.sleep(0.1)
        return True

    def _save_calibration_yaml(self, transform, filepath: str):
        """Sauvegarde le transform de calibration dans un fichier YAML."""
        data = {
            'transform': {
                'translation': {
                    'x': float(transform.translation.x),
                    'y': float(transform.translation.y),
                    'z': float(transform.translation.z),
                },
                'rotation': {
                    'x': float(transform.rotation.x),
                    'y': float(transform.rotation.y),
                    'z': float(transform.rotation.z),
                    'w': float(transform.rotation.w),
                }
            }
        }
        os.makedirs(os.path.dirname(filepath), exist_ok=True)
        with open(filepath, 'w') as f:
            yaml.dump(data, f, default_flow_style=False)
        self.get_logger().info(f'[calibration] Résultat sauvegardé dans {filepath}')

    def _traj_feedback_cb(self, feedback_msg):
        p = feedback_msg.feedback.step_progression
        self.get_logger().info(
            f'    → exécution : {p * 100:.0f}%',
            throttle_duration_sec=1.0)


# =============================================================================
# main
# =============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = AutoCalibrationNode()
    executor = MultiThreadedExecutor()
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
