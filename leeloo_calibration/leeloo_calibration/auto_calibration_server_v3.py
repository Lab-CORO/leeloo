#!/usr/bin/env python3
"""
auto_calibration_server_v2.py
=============================
Noeud ROS2  –  package : leeloo_calibration

Machine à états pilotée par un timer à 1 Hz.
Lance la calibration automatiquement dès que tous les services sont prêts.

États :
  WAITING_FOR_SERVERS → SET_PLANNER → SET_PLANNER_WAIT
  → GENERATE → GENERATE_WAIT
  → SEND_GOAL → SEND_GOAL_WAIT → EXECUTE_WAIT
  → CAPTURE → CAPTURE_WAIT
  → WAIT_BETWEEN → (GENERATE | DONE)

Usage :
    ros2 run leeloo_calibration auto_calibration_server_v2
    ros2 run leeloo_calibration auto_calibration_server_v2 \
        --ros-args -p num_poses:=10 -p wait_between_poses_s:=3.0
"""

import enum
import os

import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

# from std_srvs.srv import Trigger
from ros2_markertracker_interfaces.srv import CapturePoint
from curobo_msgs.srv import TrajectoryGeneration, SetPlanner
from curobo_msgs.action import SendTrajectory
from leeloo_msgs.action import RunCalibration

### Je pense a deux probleme: joint dans curobo et sphere collision du doosan 
_DEFAULT_POSES_FILE = '/home/ros2_ws/src/leeloo_calibration/config/calibration_poses.yaml'

# Format : (j1, j2, j4, j5, j3, j6)  — ordre réel des joints dans les messages (a verifier****)
# Valeurs en radians
# CALIBRATION_POSES = [ #Poses obtenues avec pose_saver_node (remplacé par chargement YAML)
#     (-0.0173779670,   1.500939846,  -1.2937419415, -0.8391569853,  2.2739036083,    0.7440846562),
#     (-0.0243907459,   1.662129879,  -1.7188413143, 2.01105189320,  -2.2210028172,  -0.778598249),
#     (-0.1653511077,  -4.4190583229, -1.7187250853, 1.67198324200,  -2.3159906864,  -1.2796461582),
#     (-0.3716318011,  -4.9295725822, -2.2352147102, 2.04178190230,  -1.6868780851,  -0.2596800029),
#     (-0.2789320946,  -3.394857645,  1.58154904840, 2.06068515780,  -1.8278198242,  -0.0067068874),
#     (-0.0713906288,  -3.0164067745, 1.46353530880, 0.48910954590,  1.53214466570,  -0.0011496781),
#     (-0.0440339670,  -3.8447926044, 1.46372652050, 1.69829630850,  -1.4516927004,  -0.0011496781),
#     (-0.0905026719,  -5.1945848465, -0.6589309573, 2.00706553460,  -1.192761302,   -0.0011057579),
#     (-0.1032385230,  -3.8355591297, 0.70991212130, 2.05321764950,  -2.0504946709,  -0.0011315934),
#     (-0.9125065804,  -2.9500689507, 0.82268905640, 2.03779935840,  -1.4694632292,   0.031041313),
#     (-0.5879924893,  -4.4139094353, -1.5587311983, 0.98442959790,  0.8424225450,    0.0101016676),
#     (-0.8064054847,  -3.6839592457, -1.6128203869, 0.98113077880,  1.41760051250,   0.0100913337),
#     (-0.2518234551,  -2.5186710358, -2.0304532051, 0.76453024150,  1.5180941820,    0.0101094181),
#     (-0.2932945192,  -5.8750677109, 2.2104773521,  0.75254476070,  1.18230569360,  -2.0444972515),
#     (-0.4205724597,  -5.6872358322, 1.7104344368,  1.79774236680,  0.78493744130,  -0.6095128655),
# ]

# CALIBRATION_POSES = [ # poses obtenues en regardant les topics
#     ( 0.6660589575767517,  0.1669885367155075,  2.0413761138916016,  0.9269945621490479,  -1.5414059162139893,  2.2075424194335938),
#     (-0.2599337399005890,  0.1675403416156768,  2.2437465190887450,  1.0423344373703003,  -1.2331784963607788,  2.2075371742248535),
#     (-0.2885511517524719,  0.7159800529479980,  2.1147921085357666,  1.8859941959381104,  -1.0279130935668945,  1.8152333498001099),
#     (-0.0303308367729187,  0.5259178280830383,  1.7877774238586426,  0.0413883812725544,  -2.0648660659790040,  1.9780020713806152),
#     (-0.6569235324859619,  0.2988198697566986,  1.6038537025451660,  1.5417623519897460,  -1.1396929025650024,  2.2004349231719970),
#     (-0.6569235324859619,  0.2988198697566986,  1.6038537025451660,  1.5417623519897460,  -1.1396929025650024,  2.2004349231719970),
#     (-0.4538237154483795,  1.0701659917831420,  1.3003984689712524,  0.7362720370292664,  -1.7990630865097046,  2.2004530429840090),
#     (-0.6494436860084534,  1.3104089498519897,  1.6802582740783691,  1.4577376842498780,  -1.3469862937927246,  2.2004556655883790),
#     ( 0.6484702229499817,  0.7842746973037720,  1.9440177679061890,  2.0400173664093018,  -1.2138224840164185,  2.2004659175872803),
#     ( 0.5184847116470337,  0.4626023769378662,  1.6077661514282227,  0.2597729861736297,  -1.7860832214355469,  2.2004659175872803),
#     (-0.1773759871721267,  0.0253811590373516,  1.7643746137619019,  1.6098724603652954,  -1.3165597915649414,  1.4607294797897339),
#     (-0.6972454786300659,  0.0437925271689891,  2.0511999130249023,  0.5623631477355957,  -1.4628247022628784,  2.1642911434173584),
#     (-0.3417845964431762,  0.5605992674827576,  2.2269966602325440,  2.0001325607299805,  -1.2369556427001953,  2.1349446773529053),
#     (-0.7511485815048218, -0.3259112536907196,  2.2657375335693360,  0.4365082383155823,  -0.7568499445915222,  2.1349730491638184),
#     (-0.7249452471733093, -0.2691166102886200,  2.3234293460845947,  0.0959968045353889,  -1.5124030113220215,  2.6162645816802980),
# ]

# =============================================================================
# Machine à états
# =============================================================================

class State(enum.Enum):
    WAITING_FOR_SERVERS = 'waiting_for_servers'
    SET_PLANNER         = 'set_planner'
    SET_PLANNER_WAIT    = 'set_planner_wait'
    GENERATE            = 'generate'
    GENERATE_WAIT       = 'generate_wait'
    SEND_GOAL           = 'send_goal'
    SEND_GOAL_WAIT      = 'send_goal_wait'
    EXECUTE_WAIT        = 'execute_wait'
    CAPTURE             = 'capture'
    CAPTURE_WAIT        = 'capture_wait'
    WAIT_BETWEEN        = 'wait_between'
    DONE                = 'done'


# Timeout en ticks (1 tick = 1 s) pour chaque état d'attente
TIMEOUTS = {
    State.SET_PLANNER_WAIT: 5,
    State.GENERATE_WAIT:    15,
    State.SEND_GOAL_WAIT:   10,
    State.EXECUTE_WAIT:     120,
    State.CAPTURE_WAIT:     10,
}


# =============================================================================
# Node
# =============================================================================

class AutoCalibrationNode(Node):

    def __init__(self):
        super().__init__('auto_calibration_node')

        # ---- Paramètres ROS --------------------------------------------------
        self.declare_parameter('capture_point_service', '/hand_eye_calibration/capture_point')
        self.declare_parameter('traj_generate_service', '/unified_planner/generate_trajectory')
        self.declare_parameter('traj_execute_action',   '/unified_planner/execute_trajectory')
        self.declare_parameter('set_planner_service',   '/unified_planner/set_planner')
        self.declare_parameter('calibration_poses_file', _DEFAULT_POSES_FILE)
        self.declare_parameter('num_poses',             -1)   # -1 = toutes les poses du fichier
        self.declare_parameter('wait_between_poses_s',  5.0)

        # ---- Clients ---------------------------------------------------------
        self._gen_client     = self.create_client(
            TrajectoryGeneration,
            self.get_parameter('traj_generate_service').value)
        self._planner_client = self.create_client(
            SetPlanner,
            self.get_parameter('set_planner_service').value)
        self._exec_client    = ActionClient(
            self, SendTrajectory,
            self.get_parameter('traj_execute_action').value)
        self._capture_client = self.create_client(
            CapturePoint,
            self.get_parameter('capture_point_service').value)


         # Create service to get current strategy
        self.action_client = ActionClient(
            self,
            RunCalibration,
            'auto_calibration/run_calibration'
        )

        

        # ---- Machine à états -------------------------------------------------
        self._state          : State = State.WAITING_FOR_SERVERS
        self._ticks_in_state : int   = 0
        self._future                 = None   # futur ROS2 en cours

        # ---- Données de session ----------------------------------------------
        self._poses     : list  = []
        self._total     : int   = 0
        self._pose_idx  : int   = 0
        self._completed : int   = 0
        self._failed    : int   = 0
        self._wait_s    : float = 5.0

        # ---- Boucle principale à 1 Hz ----------------------------------------
        self.create_timer(1.0, self._tick)
        self.get_logger().info('[INIT] AutoCalibrationNode démarré.')

    # =========================================================================
    # Boucle principale
    # =========================================================================

    def _tick(self):
        if self._state == State.DONE:
            return

        self._ticks_in_state += 1

        # --- Timeout pour les états d'attente ---------------------------------
        timeout = TIMEOUTS.get(self._state)
        if timeout and self._ticks_in_state > timeout:
            self.get_logger().error(
                f'[{self._state.value}] Timeout ({timeout}s) — '
                f'pose {self._pose_idx + 1} ignorée.')
            self._failed   += 1
            self._pose_idx += 1
            self._go(State.GENERATE)
            return

        # --- Dispatch ---------------------------------------------------------
        match self._state:
            case State.WAITING_FOR_SERVERS: self._s_waiting_for_servers()
            case State.SET_PLANNER:         self._s_set_planner()
            case State.SET_PLANNER_WAIT:    self._s_set_planner_wait()
            case State.GENERATE:            self._s_generate()
            case State.GENERATE_WAIT:       self._s_generate_wait()
            case State.SEND_GOAL:           self._s_send_goal()
            case State.SEND_GOAL_WAIT:      self._s_send_goal_wait()
            case State.EXECUTE_WAIT:        self._s_execute_wait()
            case State.CAPTURE:             self._s_capture()
            case State.CAPTURE_WAIT:        self._s_capture_wait()
            case State.WAIT_BETWEEN:        self._s_wait_between()

    def _go(self, state: State):
        self.get_logger().debug(f'  {self._state.value} → {state.value}')
        self._state          = state
        self._ticks_in_state = 0

    def _skip_pose(self, reason: str):
        self.get_logger().error(f'  Pose {self._pose_idx + 1} ignorée : {reason}.')
        self._failed   += 1
        self._pose_idx += 1
        self._go(State.GENERATE)

    # =========================================================================
    # Handlers d'états
    # =========================================================================

    # --- WAITING_FOR_SERVERS -------------------------------------------------

    def _s_waiting_for_servers(self):
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

        if missing:
            self.get_logger().info(
                f'[waiting] En attente : {", ".join(missing)}',
                throttle_duration_sec=5.0)
            return

        self.get_logger().info('[waiting] Tous les serveurs sont prêts.')

        self._wait_s = self.get_parameter('wait_between_poses_s').value
        poses_file   = self.get_parameter('calibration_poses_file').value
        n            = self.get_parameter('num_poses').value

        # Chargement des poses depuis le fichier YAML
        if not os.path.isfile(poses_file):
            self.get_logger().fatal(
                f'[waiting] Fichier de poses introuvable : {poses_file}')
            self._go(State.DONE)
            return

        with open(poses_file, 'r') as f:
            data = yaml.safe_load(f)

        # PATCH TEMPORAIRE : calib_order du YAML est faux (double inversion dans pose_saver_node —
        # _CALIB_ORDER conçu pour input [J1,J2,J3,J4,J5,J6] mais le topic DSR publie
        # [J1,J2,J4,J5,J3,J6]). Le champ 'positions' est lui directement du topic, donc correct.
        poses_entries = data.get('poses', [])
        if not poses_entries:
            self.get_logger().fatal(
                f'[waiting] Clé "poses" vide ou absente dans {poses_file}')
            self._go(State.DONE)
            return

        all_poses = [entry['calib_order'] for entry in poses_entries]
        self._poses = list(all_poses[:n] if 0 < n <= len(all_poses) else all_poses)
        self._total = len(self._poses)

        self.get_logger().info(
            f'[waiting] {self._total} poses chargées depuis {poses_file}')

        self.get_logger().info(
            f'=== Calibration : {self._total} poses, {self._wait_s}s entre chaque ===')
        self._go(State.SET_PLANNER)

    # --- SET_PLANNER ---------------------------------------------------------

    def _s_set_planner(self):
        req              = SetPlanner.Request()
        req.planner_type = 5
        self._future     = self._planner_client.call_async(req)
        self._go(State.SET_PLANNER_WAIT)

    def _s_set_planner_wait(self):
        if not self._future.done():
            return
        resp = self._future.result()
        match resp:
            case None:
                self.get_logger().error('[set_planner] Pas de réponse. On continue.')
            case _ if resp.success:
                self.get_logger().info('[set_planner] Planner joint-space (type=5) activé.')
            case _:
                self.get_logger().warn(f'[set_planner] Échec : {resp.message}. On continue.')
        self._go(State.GENERATE)

    # --- GENERATE ------------------------------------------------------------

    def _s_generate(self):
        if self._pose_idx >= self._total:
            self.get_logger().info(
                f'=== Calibration terminée : {self._completed}/{self._total} OK'
                + (f', {self._failed} échouées.' if self._failed
                   else ' — aucune erreur.') + ' ===')
            self._go(State.DONE)
            return

        # poses[i]['positions'] : ordre topic DSR = [j1, j2, j4, j5, j3, j6]
        pose = self._poses[self._pose_idx]
        j1, j2, j3, j4, j5, j6 = pose[0], pose[1], pose[4], pose[2], pose[3], pose[5]
        self.get_logger().info(
            f'--- Pose {self._pose_idx + 1}/{self._total} '
            f'[j1={j1:.2f} j2={j2:.2f} j3={j3:.2f} '
            f'j4={j4:.2f} j5={j5:.2f} j6={j6:.2f}] rad ---')

        req                        = TrajectoryGeneration.Request()
        req.target_joint_positions = list(pose)  # déjà en ordre calib [j1, j2, j4, j5, j3, j6]
        self._future               = self._gen_client.call_async(req)
        self._go(State.GENERATE_WAIT)

    def _s_generate_wait(self):
        if not self._future.done():
            return
        resp = self._future.result()
        match resp:
            case None:
                self._skip_pose('generate_trajectory sans réponse')
            case _ if not resp.success:
                self._skip_pose(f'generate_trajectory échoué : {resp.message}')
            case _ if not resp.trajectory:
                self._skip_pose('trajectoire vide')
            case _:
                self.get_logger().info(
                    f'  [generate] {len(resp.trajectory)} points, dt={resp.dt:.4f}s')
                self._go(State.SEND_GOAL)

    # --- SEND_GOAL -----------------------------------------------------------

    def _s_send_goal(self):
        self._future = self._exec_client.send_goal_async(
            SendTrajectory.Goal(),
            feedback_callback=self._traj_feedback_cb)
        self._go(State.SEND_GOAL_WAIT)

    def _s_send_goal_wait(self):
        if not self._future.done():
            return
        goal_handle = self._future.result()
        if goal_handle is None or not goal_handle.accepted:
            self._skip_pose('goal execute_trajectory refusé')
            return
        self._future = goal_handle.get_result_async()
        self._go(State.EXECUTE_WAIT)

    def _s_execute_wait(self):
        if not self._future.done():
            return
        res_response = self._future.result()
        match res_response:
            case None:
                self._skip_pose('execute_trajectory sans résultat')
            case _ if not res_response.result.success:
                self._skip_pose(
                    f'execute_trajectory échoué : {res_response.result.message}')
            case _:
                self.get_logger().info('  [execute] Trajectoire exécutée avec succès.')
                self._go(State.CAPTURE)

    # --- CAPTURE -------------------------------------------------------------

    def _s_capture(self):
        self._future = self._capture_client.call_async(CapturePoint.Request())
        self._go(State.CAPTURE_WAIT)

    def _s_capture_wait(self):
        if not self._future.done():
            return
        pose_num = self._pose_idx + 1
        resp     = self._future.result()
        match resp:
            case None:
                self.get_logger().error(f'  [capture] Pose {pose_num} : pas de réponse.')
            case _ if resp.success:
                self.get_logger().info(f'  [capture] Sample capturé (pose {pose_num}).')
            case _:
                self.get_logger().warn(f'  [capture] Pose {pose_num} : {resp.message}')

        self._completed += 1
        self.get_logger().info(
            f'  Pose {pose_num}/{self._total} OK ✓ '
            f'({self._completed} OK, {self._failed} KO)')
        self._pose_idx += 1
        self._go(State.WAIT_BETWEEN if self._pose_idx < self._total else State.GENERATE)

    # --- WAIT_BETWEEN --------------------------------------------------------

    def _s_wait_between(self):
        remaining = int(self._wait_s) - self._ticks_in_state
        if remaining <= 0:
            self._go(State.GENERATE)
        else:
            self.get_logger().info(f'  [wait] Prochaine pose dans {remaining}s...')

    # =========================================================================
    # Feedback action execute_trajectory
    # =========================================================================

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
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
