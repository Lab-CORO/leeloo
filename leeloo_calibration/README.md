# leeloo_calibration & leeloo_msgs

Packages ROS 2 pour la **calibration main-oeil automatisée** du robot Leeloo.  
La procédure déplace le robot sur 15 poses prédéfinies via [curobo_ros](https://github.com/Lab-CORO/curobo_ros) et capture un échantillon de calibration à chaque arrêt via [ros2_handeye_calibration](https://github.com/giuschio/ros2_handeye_calibration).

---

## Packages

| Package | Rôle |
|---|---|
| `leeloo_msgs` | Définition de l'action `RunCalibration` |
| `leeloo_calibration` | Noeud serveur qui expose et exécute l'action |

---

## Prérequis

Ces packages s'exécutent dans le **Docker `LEELOO_DOCKER_CALIBRATION`** et dépendent des packages déjà présents dans `src/` :

| Dépendance | Rôle |
|---|---|
| `curobo_ros` | Planification et exécution de trajectoires (GPU) |
| `curobo_msgs` | Interfaces `TrajectoryGeneration` + `SendTrajectory` |
| `ros2_handeye_calibration` | Noeud de calibration, expose `/hand_eye_calibration/capture_point` |
| `std_srvs` | `Trigger` (service de capture) |
| `sensor_msgs` | `JointState` (format de trajectoire) |

---

## Installation

```bash
# Depuis la racine du workspace dans le docker
cd /home/ros2_ws

# 1. Builder leeloo_msgs en premier (leeloo_calibration en dépend)
colcon build --packages-select leeloo_msgs
source install/setup.bash

# 2. Builder leeloo_calibration
colcon build --packages-select leeloo_calibration
source install/setup.bash
```

> **Ou en une seule commande** (colcon résout l'ordre automatiquement) :
> ```bash
> colcon build --packages-select leeloo_msgs leeloo_calibration
> source install/setup.bash
> ```

---

## Utilisation

### 1. Lancer les noeuds requis

Avant de démarrer `leeloo_calibration`, les noeuds suivants doivent être actifs :

```bash
# Terminal 1 — curobo_ros (planification + exécution)
ros2 launch curobo_ros gen_traj.launch.py \
    robot_config_file:=/home/ros2_ws/src/curobo_ros/curobo_doosan/src/m1013/m1013.yml

# Terminal 2 — hand-eye calibration (capture des échantillons)
ros2 launch hand_eye_calibration calibration.launch.py \
    tracking_base_frame:=<camera_frame> \
    tracking_marker_frame:=<marker_frame> \
    robot_base_frame:=base_link \
    robot_effector_frame:=tool0
```

### 2. Lancer le serveur de calibration

```bash
# Terminal 3
ros2 run leeloo_calibration auto_calibration_server
```

Sortie attendue :
```
[INFO] AutoCalibrationServer démarré.
  → ros2 action send_goal /leeloo_calibration/run ...
```

### 3. Déclencher la procédure

```bash
# Terminal 4 — lance la calibration complète sur 15 poses
ros2 action send_goal /leeloo_calibration/run \
    leeloo_msgs/action/RunCalibration \
    "{num_poses: 15, wait_between_poses_s: 2.0}" \
    --feedback
```

Exemple de feedback reçu :
```
Feedback:
  current_pose: 3
  total_poses: 15
  status: Executing trajectory
  percent_complete: 19.8

Feedback:
  current_pose: 3
  total_poses: 15
  status: Capturing calibration sample
  percent_complete: 23.1
...
Result:
  success: True
  poses_completed: 15
  poses_failed: 0
  message: 15/15 poses réussies.
```

### 4. Annuler en cours de route

```bash
ros2 action cancel_goal /leeloo_calibration/run <goal_id>
```

---

## Interface de l'action

### `leeloo_msgs/action/RunCalibration`

**Goal** (entrée) :

| Champ | Type | Description |
|---|---|---|
| `num_poses` | `int32` | Nombre de poses à exécuter. `0` = toutes (15) |
| `wait_between_poses_s` | `float64` | Délai en secondes entre chaque capture. Défaut : `2.0` |

**Feedback** (progression) :

| Champ | Type | Description |
|---|---|---|
| `current_pose` | `int32` | Numéro de la pose en cours |
| `total_poses` | `int32` | Nombre total de poses |
| `status` | `string` | Étape en cours (`Generating`, `Executing`, `Capturing`, `Done`) |
| `percent_complete` | `float32` | Avancement global en % |

**Result** (résultat final) :

| Champ | Type | Description |
|---|---|---|
| `success` | `bool` | `true` si aucune pose n'a échoué |
| `poses_completed` | `int32` | Nombre de poses réussies |
| `poses_failed` | `int32` | Nombre de poses échouées (skipped) |
| `message` | `string` | Résumé lisible |

---

## Paramètres ROS du noeud

Les noms de services/actions peuvent être remappés via des paramètres :

| Paramètre | Défaut | Description |
|---|---|---|
| `capture_point_service` | `/hand_eye_calibration/capture_point` | Service de capture des échantillons |
| `traj_generate_service` | `/unified_planner/generate_trajectory` | Service de planification curobo |
| `traj_execute_action` | `/unified_planner/execute_trajectory` | Action d'exécution curobo |

Exemple avec remappages :
```bash
ros2 run leeloo_calibration auto_calibration_server \
    --ros-args \
    -p capture_point_service:=/my_calib/capture \
    -p traj_execute_action:=/unified_planner/execute_trajectory
```

---

## Poses de calibration

Les 15 poses sont définies statiquement dans `auto_calibration_server.py` dans la constante `CALIBRATION_POSES`, organisées en 3 rangées :

| Rangée | z (m) | Nb de poses |
|---|---|---|
| Haute | 0.60 | 5 |
| Médiane | 0.45 | 5 |
| Basse | 0.30 | 5 |

> ⚠️ **Ces poses sont à adapter** selon le workspace réel de votre robot.  
> Modifier la liste `CALIBRATION_POSES` dans `leeloo_calibration/auto_calibration_server.py`.  
> Format de chaque entrée : `(x, y, z, qx, qy, qz, qw)`

---

## Architecture interne

```
Client (CLI / autre noeud)
        │  Goal: {num_poses, wait_s}
        ▼
┌─────────────────────────────────────────────────┐
│          AutoCalibrationServer                   │
│          /leeloo_calibration/run                 │
│                                                  │
│  Pour chaque pose :                              │
│                                                  │
│  1. generate_trajectory ──► curobo_ros           │
│     (Service)               retourne             │
│                             JointState[] + dt    │
│                                                  │
│  2. execute_trajectory ───► curobo_ros           │
│     (Action)                goal: JointState[]   │
│                             + dt                 │
│                             feedback:            │
│                             step_progression     │
│                                                  │
│  3. capture_point ────────► hand_eye_calibration │
│     (Service Trigger)       enregistre TF sample │
│                                                  │
│  ◄── Feedback à chaque étape                     │
└─────────────────────────────────────────────────┘
        │  Result: {success, completed, failed}
        ▼
     Client
```

### Services et actions utilisés

| Interface | Type | Nom ROS | Package |
|---|---|---|---|
| Génération trajectoire | Service | `/unified_planner/generate_trajectory` | `curobo_msgs/srv/TrajectoryGeneration` |
| Exécution trajectoire | Action | `/unified_planner/execute_trajectory` | `curobo_msgs/action/SendTrajectory` |
| Capture calibration | Service | `/hand_eye_calibration/capture_point` | `std_srvs/srv/Trigger` |

### Interfaces curobo_msgs utilisées

**`TrajectoryGeneration.srv`** (goal → trajectoire planifiée) :
```
# Request
geometry_msgs/Pose target_pose
---
# Response
bool success
string message
sensor_msgs/JointState[] trajectory
float64 dt
```

**`SendTrajectory.action`** (exécution sur le robot) :
```
# Goal
sensor_msgs/JointState[] trajectory
float64 dt
---
# Result
bool success
string message
---
# Feedback
sensor_msgs/JointState joint_command
float32 step_progression
```

---

## Structure des fichiers

```
src/
├── leeloo_msgs/
│   ├── action/
│   │   └── RunCalibration.action
│   ├── CMakeLists.txt
│   └── package.xml
│
└── leeloo_calibration/
    ├── leeloo_calibration/
    │   ├── __init__.py
    │   └── auto_calibration_server.py
    ├── resource/
    │   └── leeloo_calibration
    ├── package.xml
    └── setup.py
```

---

## Packages liés

- [`curobo_ros`](https://github.com/Lab-CORO/curobo_ros) — Planification GPU avec cuRobo
- [`curobo_msgs`](https://github.com/Lab-CORO/curobo_msgs) — Interfaces ROS 2 de curobo_ros
- [`ros2_handeye_calibration`](https://github.com/giuschio/ros2_handeye_calibration) — Calibration main-oeil Tsai-Lenz
- [`tool_box`](https://github.com/Lab-CORO/tool_box) — Dépôt parent de ce package