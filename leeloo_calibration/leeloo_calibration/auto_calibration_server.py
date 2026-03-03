#!/usr/bin/env python3
"""
auto_calibration_server.py
==========================
Noeud ROS2  –  package : leeloo_calibration

Expose l'action :
    /leeloo_calibration/run   [leeloo_msgs/action/RunCalibration]

Séquence pour chaque pose :
  1. Service  /unified_planner/generate_trajectory  → curobo_msgs/srv/TrajectoryGeneration
       réponse : trajectory (JointState[])  +  dt (float64)
  2. Action   /unified_planner/execute_trajectory   → curobo_msgs/action/SendTrajectory
       goal     : sensor_msgs/JointState[] trajectory  +  float64 dt
       feedback : joint_command (JointState)  +  step_progression (float32)
       result   : bool success  +  string message
  3. Service  /hand_eye_calibration/capture_point   → std_srvs/srv/Trigger

Usage :
    ros2 run leeloo_calibration auto_calibration_server

Lancer la procédure depuis un terminal :
    ros2 action send_goal /leeloo_calibration/run \
        leeloo_msgs/action/RunCalibration \
        "{num_poses: 15, wait_between_poses_s: 2.0}"
"""

import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

#from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from curobo_msgs.srv import TrajectoryGeneration, SetPlanner
from curobo_msgs.action import SendTrajectory
from leeloo_msgs.action import RunCalibration


# Format : (j1, j2, j4, j5, j3, j6)  — ordre réel des joints dans les messages
# Valeurs en radians
CALIBRATION_POSES = [
    (   0.6660589575767517,     0.1669885367155075,       2.0413761138916016,  0.9269945621490479,     -1.5414059162139893,     2.2075424194335938), #non
    (   -0.259933739900589,     0.16754034161567688,    2.243746519088745,  1.0423344373703003,      -1.2331784963607788,       2.2075371742248535), #oui
    (   -0.2885511517524719 ,   0.715980052947998 ,    2.1147921085357666 ,   1.8859941959381104 ,    -1.0279130935668945 ,     1.8152333498001099),#non
    (   -0.0303308367729187 ,   0.5259178280830383 ,   1.7877774238586426 ,  0.0413883812725544 ,     -2.064866065979004 ,      1.9780020713806152), #oui
    (   -0.6569235324859619 ,   0.2988198697566986 ,    1.603853702545166 ,  1.541762351989746 ,       -1.1396929025650024 ,    2.200434923171997), #oui
    (   -0.6569235324859619,    0.2988198697566986,     1.603853702545166,   1.541762351989746,        -1.1396929025650024,     2.200434923171997), # en double
    (   -0.4538237154483795,    1.070165991783142,     1.3003984689712524,    0.7362720370292664,      -1.7990630865097046,     2.200453042984009),#oui
    (   -0.6494436860084534,    1.3104089498519897,    1.6802582740783691,    1.457737684249878,      -1.3469862937927246,      2.200455665588379),
    (   0.6484702229499817,     0.784274697303772,     1.944017767906189,     2.0400173664093018,       -1.2138224840164185,    2.2004659175872803),
    (   0.5184847116470337,     0.4626023769378662,     1.6077661514282227,    0.25977298617362976,    -1.7860832214355469,     2.2004659175872803),
    (   -0.17737598717212677,   0.02538115903735161,    1.7643746137619019,     1.6098724603652954,    -1.3165597915649414,     1.4607294797897339),
    (   -0.6972454786300659,    0.04379252716898918,     2.0511999130249023,   0.5623631477355957,    -1.4628247022628784,      2.1642911434173584),
    (   -0.34178459644317627,   0.5605992674827576,        2.226996660232544,   2.0001325607299805,     -1.2369556427001953,    2.1349446773529053),
    (   -0.7511485815048218,    -0.3259112536907196,     2.265737533569336,    0.4365082383155823,     -0.7568499445915222,     2.1349730491638184),
    (   -0.7249452471733093,    -0.26911661028862,      2.3234293460845947,    0.09599680453538895,      -1.5124030113220215,   2.616264581680298),
]
#metre dna sun yaml


class AutoCalibrationServer(Node):

    def __init__(self):
        super().__init__('auto_calibration_server')

        # ---- Paramètres ROS --------------------------------------------------
        self.declare_parameter(
            'capture_point_service', '/hand_eye_calibration/capture_point')
        self.declare_parameter(
            'traj_generate_service', '/unified_planner/generate_trajectory')
        self.declare_parameter(
            'traj_execute_action',   '/unified_planner/execute_trajectory')
        self.declare_parameter(
            'set_planner_service',   '/unified_planner/set_planner')

        capture_srv  = self.get_parameter('capture_point_service').value
        gen_srv      = self.get_parameter('traj_generate_service').value
        exec_act     = self.get_parameter('traj_execute_action').value
        set_plan_srv = self.get_parameter('set_planner_service').value

        # ---- Callback group réentrant (obligatoire pour spin_until_future) ---
        cb = ReentrantCallbackGroup()

        # ---- Clients curobo_ros ----------------------------------------------
        self._gen_client = self.create_client(
            TrajectoryGeneration, gen_srv, callback_group=cb)

        self._set_planner_client = self.create_client(
            SetPlanner, set_plan_srv, callback_group=cb)

        self._exec_client = ActionClient(
            self, SendTrajectory, exec_act, callback_group=cb)

        # ---- Client hand-eye calibration ------------------------------------
        self._capture_client = self.create_client(
            Trigger, capture_srv, callback_group=cb)

        # ---- Serveur d'action exposé ----------------------------------------
        self._action_server = ActionServer(
            self,
            RunCalibration,
            'leeloo_calibration/run',
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=cb,
        )

        self.get_logger().info(
            'AutoCalibrationServer démarré.\n'
            '  → ros2 action send_goal /leeloo_calibration/run \\\n'
            '      leeloo_msgs/action/RunCalibration \\\n'
            '      "{num_poses: 15, wait_between_poses_s: 2.0}"'
        )

    # =========================================================================
    # Callbacks du serveur d'action
    # =========================================================================

    def _goal_cb(self, goal_request):
        self.get_logger().info(
            f'Goal reçu — {goal_request.num_poses or len(CALIBRATION_POSES)} poses, '
            f'attente {goal_request.wait_between_poses_s}s entre chaque.')
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        self.get_logger().info('Annulation demandée.')
        return CancelResponse.ACCEPT

    # =========================================================================
    # Exécution du goal
    # =========================================================================

    def _execute_cb(self, goal_handle):
        # --- Résolution des paramètres du goal --------------------------------
        n      = goal_handle.request.num_poses
        wait_s = goal_handle.request.wait_between_poses_s or 2.0
        poses  = CALIBRATION_POSES[:n] if (0 < n <= len(CALIBRATION_POSES)) \
                 else CALIBRATION_POSES
        total     = len(poses)
        completed = 0
        failed    = 0

        self.get_logger().info(
            f'=== Démarrage calibration : {total} poses, '
            f'{wait_s}s entre chaque ===')

        # --- Attente que tous les serveurs soient prêts -----------------------
        self._publish_fb(goal_handle, 0, total, 'Waiting for servers', 0.0)
        self._wait_for_servers()

        # --- Boucle principale ------------------------------------------------
        for idx, (j1, j2, j4, j5, j3, j6) in enumerate(poses):

            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal annulé par le client.')
                goal_handle.canceled()
                return self._make_result(False, completed, failed,
                                         f'Annulé après {completed} poses.')

            pose_num = idx + 1
            self.get_logger().info(
                f'--- Pose {pose_num}/{total} '
                f'[j1={j1:.2f}, j2={j2:.2f}, j3={j3:.2f}, '
                f'j4={j4:.2f}, j5={j5:.2f}, j6={j6:.2f}] rad ---')

            # 1) Génération trajectoire ----------------------------------------
            self._publish_fb(goal_handle, pose_num, total,
                             'Generating trajectory',
                             self._pct(idx, total, 0.0))

            traj_msg, dt = self._generate_trajectory(j1, j2, j4, j5, j3, j6)
            if traj_msg is None:
                self.get_logger().error(
                    f'  Génération échouée (pose {pose_num}). Skipped.')
                failed += 1
                continue

            # 2) Exécution trajectoire -----------------------------------------
            self._publish_fb(goal_handle, pose_num, total,
                             'Executing trajectory',
                             self._pct(idx, total, 0.33))

            if not self._execute_trajectory(traj_msg, dt):
                self.get_logger().error(
                    f'  Exécution échouée (pose {pose_num}). Skipped.')
                failed += 1
                continue

            # 3) Capture calibration -------------------------------------------
            self._publish_fb(goal_handle, pose_num, total,
                             'Capturing calibration sample',
                             self._pct(idx, total, 0.66))

            self._capture_sample(pose_num)
            completed += 1

            self._publish_fb(goal_handle, pose_num, total,
                             f'Pose {pose_num} done ✓',
                             self._pct(idx + 1, total, 0.0))

            if idx < total - 1:
                time.sleep(wait_s)

        # --- Résultat final ---------------------------------------------------
        self.get_logger().info(
            f'=== Calibration terminée : {completed}/{total} OK, '
            f'{failed} échouées ===')
        goal_handle.succeed()
        return self._make_result(
            failed == 0, completed, failed,
            f'{completed}/{total} poses réussies'
            + (f', {failed} échouées.' if failed else '.'))

    # =========================================================================
    # Helpers internes
    # =========================================================================

    @staticmethod
    def _pct(idx, total, offset):
        return min(100.0, (idx / total + offset / total) * 100.0)

    def _publish_fb(self, gh, current, total, status, pct):
        fb = RunCalibration.Feedback()
        fb.current_pose     = current
        fb.total_poses      = total
        fb.status           = status
        fb.percent_complete = float(pct)
        gh.publish_feedback(fb)
        self.get_logger().info(f'  [{pct:5.1f}%]  {status}')

    @staticmethod
    def _make_result(success, completed, failed, message):
        r = RunCalibration.Result()
        r.success         = success
        r.poses_completed = completed
        r.poses_failed    = failed
        r.message         = message
        return r

    # -----------------------------------------------------------------------

    def _wait_for_servers(self):
        for client, name in [
            (self._gen_client,         'generate_trajectory'),
            (self._set_planner_client, 'set_planner'),
            (self._capture_client,     'capture_point'),
        ]:
            while not client.wait_for_service(timeout_sec=2.0):
                self.get_logger().warn(f'  En attente de {name}...')
        while not self._exec_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('  En attente de execute_trajectory (action)...')
        self.get_logger().info('  Tous les serveurs sont prêts.')
        self._set_joint_space_planner()

    def _set_joint_space_planner(self):
        """Passe curobo en mode joint-space (planner_type=5)."""
        req = SetPlanner.Request()
        req.planner_type = 5
        future = self._set_planner_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() is None:
            self.get_logger().error('  set_planner : timeout.')
        elif future.result().success:
            self.get_logger().info('  Planner joint-space (type=5) activé.')
        else:
            self.get_logger().warn(
                f'  set_planner a échoué : {future.result().message}')

    # -----------------------------------------------------------------------

    def _generate_trajectory(self, j1, j2, j4, j5, j3, j6):
        """
        Appelle generate_trajectory en mode joint-space.
        L'ordre des joints dans le message est : j1, j2, j4, j5, j3, j6
        Retourne (JointState[], dt) ou (None, None) en cas d'erreur.
        """
        req = TrajectoryGeneration.Request()
        req.target_joint_positions = [j1, j2, j4, j5, j3, j6]

        future = self._gen_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)

        if future.result() is None:
            self.get_logger().error('  generate_trajectory : timeout.')
            return None, None

        resp = future.result()
        if not resp.success:
            self.get_logger().error(
                f'  generate_trajectory a échoué : {resp.message}')
            return None, None

        if not resp.trajectory:
            self.get_logger().error(
                '  generate_trajectory : trajectoire vide.')
            return None, None

        self.get_logger().info(
            f'  Trajectoire générée : {len(resp.trajectory)} points, '
            f'dt={resp.dt:.4f}s')
        return resp.trajectory, resp.dt

    # -----------------------------------------------------------------------

    def _execute_trajectory(self, trajectory: list, dt: float) -> bool:
        """
        Envoie la trajectoire via l'action execute_trajectory.
        SendTrajectory.Goal est vide — la trajectoire est déjà stockée
        côté serveur par l'appel précédent à generate_trajectory.

        Note : on utilise threading.Event pour attendre le résultat plutôt
        que spin_until_future_complete, car ce dernier ne fonctionne pas
        correctement depuis un callback du même MultiThreadedExecutor pour
        des attentes longues (retourne prématurément après ~5 s).
        Les autres threads de l'executor traitent les messages en arrière-plan.
        """
        try:
            goal_msg = SendTrajectory.Goal()

            # -- Envoi du goal et attente acceptation (réponse rapide ≤ 10 s) --
            send_future = self._exec_client.send_goal_async(
                goal_msg,
                feedback_callback=self._send_traj_feedback_cb)
            rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)

            gh = send_future.result()
            if gh is None or not gh.accepted:
                self.get_logger().error('  execute_trajectory : goal refusé.')
                return False

            # -- Attente du résultat via threading.Event --
            # spin_until_future_complete retourne prématurément quand il est
            # appelé depuis un thread de l'executor (contention sur le wait-set).
            # Avec Event.wait(), Thread A se bloque et les threads B/C/D de
            # l'executor traitent la réponse en arrière-plan.
            done_event = threading.Event()
            result_holder = [None]

            def _on_result(future):
                result_holder[0] = future.result()
                done_event.set()

            result_future = gh.get_result_async()
            result_future.add_done_callback(_on_result)

            if not done_event.wait(timeout=120.0):
                self.get_logger().error(
                    '  execute_trajectory : timeout résultat (120 s).')
                return False

            res_response = result_holder[0]
            if res_response is None:
                self.get_logger().error('  execute_trajectory : résultat None.')
                return False

            res = res_response.result
            if not res.success:
                self.get_logger().error(
                    f'  execute_trajectory échoué : {res.message}')
                return False

            self.get_logger().info('  Trajectoire exécutée avec succès.')
            return True

        except Exception as e:
            self.get_logger().error(f'  execute_trajectory EXCEPTION : {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            return False

    def _send_traj_feedback_cb(self, feedback_msg):
        try:
            p = feedback_msg.feedback.step_progression
            self.get_logger().info(
                f'    → exécution : {p * 100:.0f}%',
                throttle_duration_sec=1.0)
        except Exception as e:
            self.get_logger().warn(f'    feedback_cb erreur : {e}')

# =============================================================================
# main
# =============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = AutoCalibrationServer()
    # MultiThreadedExecutor indispensable : le serveur d'action tourne dans
    # un thread pendant que les spin_until_future_complete tournent dans un autre.
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
