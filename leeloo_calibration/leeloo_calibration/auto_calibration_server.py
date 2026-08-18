#!/usr/bin/env python3
"""
auto_calibration_server_v2.py
=============================
Noeud ROS2    package : leeloo_calibration

Action server RunCalibration (leeloo_msgs/action/RunCalibration).
La calibration démarre uniquement sur réception d'un goal.

L'execute_callback pilote directement la procédure (pas de timer externe).
Les appels de services sont non-bloquants (call_async) et attendus par
polling (_wait_future), ce qui permet au spin de continuer à tourner.

Le goal sélectionne la caméra à calibrer via `handeye_server` (ex: 'kinect',
'oak') : les clients de service vers ce handeye_server sont construits à la
volée et mis en cache (_get_clients), sans registre de profils à déclarer par
caméra. Ajouter une caméra = lancer son handeye_server remappé sur
/easy_handeye2_<nom>/calibration/* (voir leeloo_calibration/launch/
calibration.launch.py) ; rien à modifier ici.

Usage :
    ros2 run leeloo_calibration auto_calibration_server_v2
"""

import os
import time
from dataclasses import dataclass
from datetime import datetime

import yaml
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter_client import AsyncParameterClient

from std_srvs.srv import Trigger
from easy_handeye2_msgs.srv import TakeSample, RemoveSample, ComputeCalibration, SaveCalibration
from curobo_msgs.srv import TrajectoryGeneration, SetPlanner
from curobo_msgs.action import SendTrajectory
from leeloo_msgs.action import RunCalibration
from ros2_markertracker_interfaces.msg import FiducialMarkerArray

_CALIB_CONFIG_DIR   = os.path.join(get_package_share_directory('leeloo_calibration'), 'config')
_DEFAULT_POSES_FILE = os.path.join(_CALIB_CONFIG_DIR, 'calibration_poses.yaml')
_DEFAULT_WAIT_S     = 5.0
_RESULT_DIR         = _CALIB_CONFIG_DIR


@dataclass
class _HandeyeClients:
    """Clients vers UN handeye_server, résolus à partir du champ `handeye_server`
    du goal (ex: 'kinect' -> /easy_handeye2_kinect/calibration/*)."""
    take_sample: object
    get_sample_list: object
    remove_sample: object
    compute_calibration: object


class AutoCalibrationNode(Node):

    def __init__(self):
        super().__init__('auto_calibration_node')
        # ---- Paramètres ROS --------------------------------------------------
        self.declare_parameter('traj_generate_service', '/curobo_trajectory_planner/generate_trajectory')
        self.declare_parameter('traj_execute_action',   '/curobo_trajectory_planner/execute_trajectory')
        self.declare_parameter('set_planner_service',   '/curobo_trajectory_planner/set_planner')
        self.declare_parameter('calibration_poses_file', _DEFAULT_POSES_FILE)
        self.declare_parameter('calibration_result_file',
            os.path.join(_CALIB_CONFIG_DIR, 'kinect_hand_eye_result.yaml'))
        self.declare_parameter('settle_time_s', 2.0)
        self.declare_parameter('reload_calibration_service',
            '/kinect_tf_computation_node/reload_calibration')
        # Nom du handeye_server utilisé quand le goal laisse `handeye_server` vide
        # (rétrocompatibilité : reproduit exactement le comportement d'avant ce
        # champ). Le rechargement TF automatique (reload_calibration_service)
        # ne s'applique QUE quand ce handeye_server est celui ciblé — voir
        # "ce que ce lot ne fait pas" dans le plan : pas de nœud de reload
        # générique pour les autres caméras dans ce lot.
        self.declare_parameter('default_handeye_server', 'kinect')
        # Borne l'attente des services d'un handeye_server -- sans registre de
        # noms connus, une faute de frappe sur `handeye_server` ne peut plus
        # être détectée à l'avance : sans ce délai, la boucle d'attente des
        # serveurs bloquerait indéfiniment.
        self.declare_parameter('server_wait_timeout_s', 30.0)

        # ---- Paramètres debug capture -------------------------------------------
        self.declare_parameter('enable_debug_capture', False)
        self.declare_parameter('debug_capture_file',
            os.path.join(_CALIB_CONFIG_DIR, 'calibration_debug_samples.yaml'))
        # '/fiducial_markers' est un nom de topic ABSOLU dans markertracker_node.py
        # (_result_marker_topic = '/fiducial_markers') -> le namespace='/ros2_markertracker'
        # du launch file n'a AUCUN effet dessus (seuls les noms relatifs sont préfixés
        # par le namespace d'un Node en ROS2). Le topic réel reste '/fiducial_markers'.
        self.declare_parameter('fiducial_markers_topic', '/fiducial_markers')
        self.declare_parameter('marker_id_for_ambiguity', 18)

        # ---- Callback group pour les clients ---------------------------------
        # ReentrantCallbackGroup requis : l'execute_callback de l'action server
        # tourne dans son propre thread et doit pouvoir recevoir les réponses
        # de services en parallèle (sinon deadlock avec MutuallyExclusiveCallbackGroup).
        self._clients_cb_group = ReentrantCallbackGroup()

        # ---- Clients curobo (partagés, pas spécifiques à une caméra) ---------
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
        self._reload_client  = self.create_client(
            Trigger,
            self.get_parameter('reload_calibration_service').value,
            callback_group=self._clients_cb_group)

        # ---- Clients easy_handeye2, construits à la volée par handeye_server -
        self._client_cache: dict[str, _HandeyeClients] = {}

        # ---- Debug capture init ------------------------------------------------
        self._enable_debug_capture = self.get_parameter('enable_debug_capture').value
        self._debug_capture_file = self.get_parameter('debug_capture_file').value
        self._fiducial_markers_topic = self.get_parameter('fiducial_markers_topic').value
        self._marker_id_for_ambiguity = self.get_parameter('marker_id_for_ambiguity').value
        self._debug_points = []
        self._latest_marker_info = {}

        if self._enable_debug_capture:
            self._fiducial_markers_sub = self.create_subscription(
                FiducialMarkerArray,
                self._fiducial_markers_topic,
                self._on_fiducial_markers,
                10,
                callback_group=self._clients_cb_group)
            self.get_logger().info(f'[DEBUG] Debug capture ACTIVÉ — markers topic: {self._fiducial_markers_topic}')
        else:
            self._fiducial_markers_sub = None
            self.get_logger().info('[DEBUG] Debug capture désactivé.')

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
    # Clients easy_handeye2 (à la volée, mis en cache par handeye_server)
    # =========================================================================

    def _get_clients(self, handeye_server: str) -> _HandeyeClients:
        """Construit (une fois) les clients vers /easy_handeye2_<handeye_server>/calibration/*.

        Aucune déclaration préalable requise : ajouter une caméra = lancer son
        handeye_server avec ce remap (calibration.launch.py), rien d'autre à
        modifier ici.
        """
        if handeye_server not in self._client_cache:
            ns = f'/easy_handeye2_{handeye_server}/calibration/'
            self._client_cache[handeye_server] = _HandeyeClients(
                take_sample=self.create_client(
                    TakeSample, ns + 'take_sample', callback_group=self._clients_cb_group),
                get_sample_list=self.create_client(
                    TakeSample, ns + 'get_sample_list', callback_group=self._clients_cb_group),
                remove_sample=self.create_client(
                    RemoveSample, ns + 'remove_sample', callback_group=self._clients_cb_group),
                compute_calibration=self.create_client(
                    ComputeCalibration, ns + 'compute_calibration', callback_group=self._clients_cb_group),
            )
        return self._client_cache[handeye_server]

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

        self._running est toujours remis à False en sortie (même en cas
        d'exception) pour ne jamais bloquer les goals suivants.
        """
        self._running = True
        try:
            return self._execute_cb_impl(goal_handle)
        except Exception as e:
            self.get_logger().error(f'[action] Exception dans execute_callback : {e}')
            result                  = RunCalibration.Result()
            result.success          = False
            result.poses_completed  = 0
            result.poses_failed     = 0
            result.message          = f'Erreur interne : {e}'
            if goal_handle.is_active:
                goal_handle.abort()
            return result
        finally:
            self._running = False

    def _execute_cb_impl(self, goal_handle):
        n    = goal_handle.request.num_poses
        wait = goal_handle.request.wait_between_poses_s or _DEFAULT_WAIT_S
        n_eff = n if n > 0 else -1

        # ---- Résolution des sélecteurs du goal (tous optionnels, replis legacy) ---
        handeye_server   = goal_handle.request.handeye_server or self.get_parameter('default_handeye_server').value
        poses_file       = goal_handle.request.poses_file or self.get_parameter('calibration_poses_file').value
        camera_topic     = goal_handle.request.camera_topic
        calibration_type = goal_handle.request.calibration_type

        if camera_topic:
            slug = camera_topic.strip('/').replace('/', '_')
            result_file = os.path.join(_RESULT_DIR, f'{slug}_hand_eye_result.yaml')
        else:
            result_file = self.get_parameter('calibration_result_file').value

        clients = self._get_clients(handeye_server)

        self.get_logger().info(
            f"[action] Goal accepté — handeye_server='{handeye_server}', "
            f"num_poses={n}, wait={wait}s, calibration_type='{calibration_type or '?'}'")

        result                 = RunCalibration.Result()
        result.poses_completed = 0
        result.poses_failed    = 0

        # ---- 1. Attente des serveurs -----------------------------------------
        self.get_logger().info('[action] Attente des serveurs...')
        waited = 0.0
        while True:
            missing = self._check_servers(clients)
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
                return result
            waited += 1.0
            if waited > self.get_parameter('server_wait_timeout_s').value:
                result.success = False
                result.message = (
                    f"Serveur(s) indisponible(s) après {waited:.0f}s : {missing}. "
                    f"Vérifie que le handeye_server '{handeye_server}' est bien lancé "
                    f"(attendu : /easy_handeye2_{handeye_server}/calibration/...).")
                self.get_logger().fatal(result.message)
                goal_handle.abort()
                return result

        self.get_logger().info('[action] Tous les serveurs sont prêts.')

        # ---- 1a. Validation croisée du type de calibration (si fourni) -------
        if calibration_type:
            node_name = f'{handeye_server}_handeye_server'
            live = self._query_handeye_params(node_name, ['calibration_type'], goal_handle=goal_handle)
            live_type = live.get('calibration_type')
            if live_type and live_type != calibration_type:
                result.success = False
                result.message = (
                    f"calibration_type='{calibration_type}' demandé, mais {node_name} "
                    f"est configuré en '{live_type}'. Calibration annulée.")
                self.get_logger().fatal(f'[action] {result.message}')
                goal_handle.abort()
                return result
            elif not live_type:
                self.get_logger().warn(
                    f'[action] Impossible de lire calibration_type sur {node_name} — '
                    f'validation croisée ignorée.')

        # ---- 1b. Init debug capture (si activé) --------------------------------
        self._debug_points = []
        self._handeye_parameters = {}
        if self._enable_debug_capture:
            # Archiver l'ancien fichier si existe
            if os.path.isfile(self._debug_capture_file):
                ts = datetime.now().strftime('%Y%m%d_%H%M%S')
                archive_file = self._debug_capture_file.replace('.yaml', f'_{ts}.yaml')
                try:
                    os.rename(self._debug_capture_file, archive_file)
                    self.get_logger().info(f'[DEBUG] Ancien fichier archivé: {archive_file}')
                except Exception as e:
                    self.get_logger().warn(f'[DEBUG] Archivage échoué: {e}')

            # Récupérer les paramètres de handeye_server
            node_name = f'{handeye_server}_handeye_server'
            param_names = ['calibration_type', 'robot_base_frame', 'robot_effector_frame',
                           'tracking_base_frame', 'tracking_marker_frame']
            self._handeye_parameters = self._query_handeye_params(
                node_name, param_names, goal_handle=goal_handle)
            if self._handeye_parameters:
                self.get_logger().info(f'[DEBUG] Paramètres {node_name}: OK')
            else:
                self.get_logger().warn(f'[DEBUG] Timeout récupération paramètres {node_name}')

        # ---- 1c. Reset des samples hand-eye ---------------------------------
        list_resp = self._wait_future(
            clients.get_sample_list.call_async(TakeSample.Request()),
            timeout=5.0, goal_handle=goal_handle)
        if list_resp is not None:
            num_samples = len(list_resp.samples.samples)
            for i in range(num_samples):
                remove_resp = self._wait_future(
                    clients.remove_sample.call_async(RemoveSample.Request(sample_index=0)),
                    timeout=5.0, goal_handle=goal_handle)
                if remove_resp is None:
                    self.get_logger().warn(f'[action] Reset : remove_sample({i}) timeout.')
            self.get_logger().info(f'[action] Samples réinitialisés ({num_samples} supprimés).')
        else:
            self.get_logger().warn('[action] Reset samples : get_sample_list timeout ou erreur.')

        # ---- 2. Chargement des poses -----------------------------------------
        poses, error = self._load_poses(poses_file, n_eff)
        if error:
            self.get_logger().fatal(error)
            result.success = False
            result.message = error
            goal_handle.abort()
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

        for i, pose in enumerate(poses):

            # Vérification annulation en début de pose
            if goal_handle.is_cancel_requested:
                result.success         = False
                result.poses_completed = completed
                result.poses_failed    = failed
                result.message         = f'Annulé à la pose {i + 1}/{total}.'
                goal_handle.canceled()
                return result

            j1, j2, j3, j4, j5, j6 = pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]
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
            exec_goal = SendTrajectory.Goal()
            exec_goal.target_joint_positions = list(pose)
            traj_gh_future = self._exec_client.send_goal_async(
                exec_goal,
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
            # self.get_logger().info('  [execute] Trajectoire exécutée avec succès.')

            # -- Stabilisation du robot avant capture --------------------------
            settle = self.get_parameter('settle_time_s').value
            # self.get_logger().info(f'  [settle] Attente stabilisation {settle:.1f}s...')
            if not self._sleep_or_cancel(settle, goal_handle):
                result.success         = False
                result.poses_completed = completed
                result.poses_failed    = failed
                result.message         = f'Annulé pendant stabilisation à la pose {i + 1}.'
                goal_handle.canceled()
                return result

            # -- Capture -------------------------------------------------------
            _fb('Capturing')
            cap_resp = self._wait_future(
                clients.take_sample.call_async(TakeSample.Request()),
                timeout=10.0, goal_handle=goal_handle)

            if cap_resp is None:
                self.get_logger().error(f'  [capture] Pose {i + 1} : timeout ou annulé.')
            else:
                sample_count = len(cap_resp.samples.samples)
                # self.get_logger().info(f'  [capture] Sample capturé (pose {i + 1}, total={sample_count}).')

                # ── Debug capture ────────────────────────────────────────────
                if self._enable_debug_capture and sample_count > 0:
                    try:
                        # Récupérer le dernier sample qui vient d'être capturé
                        latest_sample = cap_resp.samples.samples[-1]

                        # Extraire les transforms
                        robot_tf = latest_sample.robot
                        tracking_tf = latest_sample.tracking

                        debug_point = {
                            'index': i + 1,
                            'timestamp': datetime.now().isoformat(),
                            'joint_positions': list(pose),
                            'robot_transform': {
                                'translation': {
                                    'x': float(robot_tf.translation.x),
                                    'y': float(robot_tf.translation.y),
                                    'z': float(robot_tf.translation.z),
                                },
                                'rotation': {
                                    'x': float(robot_tf.rotation.x),
                                    'y': float(robot_tf.rotation.y),
                                    'z': float(robot_tf.rotation.z),
                                    'w': float(robot_tf.rotation.w),
                                },
                            },
                            'tracking_transform': {
                                'translation': {
                                    'x': float(tracking_tf.translation.x),
                                    'y': float(tracking_tf.translation.y),
                                    'z': float(tracking_tf.translation.z),
                                },
                                'rotation': {
                                    'x': float(tracking_tf.rotation.x),
                                    'y': float(tracking_tf.rotation.y),
                                    'z': float(tracking_tf.rotation.z),
                                    'w': float(tracking_tf.rotation.w),
                                },
                            },
                            'ambiguity_ratio': float(self._latest_marker_info.get('ambiguity_ratio', float('inf'))),
                            'reproj_error': float(self._latest_marker_info.get('reproj_error', float('nan'))),
                        }
                        self._debug_points.append(debug_point)
                        self._save_debug_points_yaml()
                    except Exception as e:
                        self.get_logger().warn(f'[DEBUG] Erreur capture point {i + 1}: {e}')

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
                    return result

        # ---- 5. Calcul de la calibration (après toutes les poses) -----------
        self.get_logger().info(
            f'=== Toutes les poses capturées : {completed}/{total} OK'
            + (f', {failed} échouées.' if failed else ' — aucune erreur.') + ' ===')

        # ---- 5b. Calcul et sauvegarde du résultat --------------------------
        calib_resp = self._wait_future(
            clients.compute_calibration.call_async(ComputeCalibration.Request()),
            timeout=10.0, goal_handle=goal_handle)

        if calib_resp is not None and calib_resp.valid:
            self.get_logger().info('[calibration] Calibration calculée avec succès.')
            self._save_calibration_yaml(
                calib_resp.calibration.transform, result_file,
                handeye_server=handeye_server, camera_topic=camera_topic,
                calibration_type=calibration_type)

            default_he = self.get_parameter('default_handeye_server').value
            if handeye_server != default_he:
                # Décision : pas de nœud de reload générique dans ce lot pour les
                # caméras autres que celle par défaut — le résultat est calculé et
                # sauvegardé, l'application en TF live reste manuelle.
                self.get_logger().info(
                    f"[calibration] Aucun rechargement TF configuré pour "
                    f"'{handeye_server}' — résultat sauvegardé dans {result_file}. "
                    f"À appliquer manuellement.")
            elif self._reload_client.service_is_ready():
                reload_resp = self._wait_future(
                    self._reload_client.call_async(Trigger.Request()), timeout=5.0)
                if reload_resp is not None and reload_resp.success:
                    self.get_logger().info(
                        '[calibration] kinect_tf_computation_node rechargé avec la nouvelle calibration.')
                else:
                    self.get_logger().warn(
                        '[calibration] Échec du rechargement kinect_tf_computation_node.')
            else:
                self.get_logger().warn(
                    '[calibration] Service reload_calibration non disponible — '
                    'redémarre kinect_tf_computation_node pour appliquer la nouvelle calibration.')
        else:
            self.get_logger().warn(
                '[calibration] Calcul échoué ou pas assez de samples valides.')

        result.success         = (calib_resp is not None and calib_resp.valid)
        result.poses_completed = completed
        result.poses_failed    = failed
        result.message         = (
            f'Calibration terminée : {completed}/{total} OK'
            + (f', {failed} échouées.' if failed else '.')
        )
        if calib_resp is not None and calib_resp.valid:
            result.transform = calib_resp.calibration.transform
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    # =========================================================================
    # Helpers
    # =========================================================================

    def _check_servers(self, clients: _HandeyeClients) -> list:
        """Retourne la liste des serveurs non encore disponibles."""
        missing = [
            name for client, name in [
                (self._gen_client,     'generate_trajectory'),
                (self._planner_client, 'set_planner'),
                (clients.take_sample,  'take_sample'),
            ]
            if not client.service_is_ready()
        ]
        if not self._exec_client.server_is_ready():
            missing.append('execute_trajectory')
        return missing

    def _query_handeye_params(self, node_name: str, param_names: list,
                               timeout: float = 3.0, goal_handle=None) -> dict:
        """Lit une liste de paramètres string sur un handeye_server distant.

        Retourne {} en cas de timeout ou d'erreur — appelant responsable de
        décider si c'est bloquant ou juste un avertissement.
        """
        try:
            param_client = AsyncParameterClient(self, node_name)
            future = param_client.get_parameters(param_names)
            params_result = self._wait_future(future, timeout=timeout, goal_handle=goal_handle)
            if not params_result:
                return {}
            return {p.name: p.value.string_value for p in params_result.values}
        except Exception as e:
            self.get_logger().warn(f'[params] Erreur lecture paramètres {node_name}: {e}')
            return {}

    def _load_poses(self, poses_file: str, n: int):
        """
        Charge les poses depuis le fichier YAML.
        Retourne (poses, None) ou (None, message_erreur).
        """
        if not os.path.isfile(poses_file):
            return None, f'Fichier de poses introuvable : {poses_file}'
        with open(poses_file, 'r') as f:
            data = yaml.safe_load(f)
        poses_entries = data.get('poses', [])
        if not poses_entries:
            return None, f'Clé "poses" vide ou absente dans {poses_file}'
        all_poses = [entry['positions'] for entry in poses_entries]
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
            # if i % 10 == 0:
            #     self.get_logger().info(f'  [wait] Prochaine pose dans {remaining:.0f}s...')
            time.sleep(0.1)
        return True

    def _save_debug_points_yaml(self):
        """Sauvegarde les points de debug capturés dans un fichier YAML."""
        if not self._debug_points:
            return
        data = {
            'metadata': {
                'handeye_parameters': self._handeye_parameters,
                'total_points': len(self._debug_points),
            },
            'points': self._debug_points,
        }
        os.makedirs(os.path.dirname(self._debug_capture_file), exist_ok=True)
        with open(self._debug_capture_file, 'w') as f:
            yaml.dump(data, f, default_flow_style=False)

    def _save_calibration_yaml(self, transform, filepath: str,
                                handeye_server: str = '', camera_topic: str = '',
                                calibration_type: str = ''):
        """Sauvegarde le transform de calibration dans un fichier YAML.

        `metadata` est purement informatif : kinect_tf_computation_node ne lit
        que la clé `transform`, donc cet ajout est rétrocompatible.
        """
        data = {
            'metadata': {
                'handeye_server': handeye_server,
                'camera_topic': camera_topic,
                'calibration_type': calibration_type,
                'saved_at': datetime.now().isoformat(timespec='seconds'),
            },
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
        # self.get_logger().info(f'[calibration] Résultat sauvegardé dans {filepath}')

    def _on_fiducial_markers(self, msg):
        """Callback pour FiducialMarkerArray — cache les infos ambiguity du marqueur pertinent."""
        if not msg.marker:
            return
        for marker in msg.marker:
            if marker.id == self._marker_id_for_ambiguity:
                self._latest_marker_info = {
                    'ambiguity_ratio': marker.ambiguity_ratio,
                    'reproj_error': marker.reproj_error,
                    'stamp': msg.camera_frame_stamp,
                }
                break

    def _traj_feedback_cb(self, feedback_msg):
        p = feedback_msg.feedback.step_progression
        # self.get_logger().info(
        #     f'    → exécution : {p * 100:.0f}%',
        #     throttle_duration_sec=1.0)


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
