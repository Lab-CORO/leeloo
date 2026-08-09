# leeloo_gazebo

Simulation Gazebo Sim (Harmonic / gz-sim 8, ROS 2 Jazzy) de **Leeloo** — Clearpath Ridgeback
+ Doosan M1013 + Robotiq 2F-85 + caméras de profondeur — construite à partir de
`leeloo/urdf/leeloo.urdf`.

Ce paquet n'est **pas** une seconde pile logicielle : c'est un **substitut de matériel**.
Chaque topic qu'il expose porte le nom et le type de son équivalent réel, si bien que
`curobo_ros`, `robot_segmentation`, `execute_trajectory`, `ridgeback_control` et `grasp`
tournent **sans aucune modification** contre la simulation. Seuls les *producteurs*
changent.

Le paquet `leeloo` n'est jamais modifié par ce fichier de lancement : `leeloo.urdf` est
patché **en mémoire** (voir [Chirurgie XML](#chirurgie-xml-sur-leeloourdf)).

---

## Démarrage rapide

### Pile complète (le cas normal)

```bash
cd /home/ros2_ws
colcon build --packages-select leeloo_msgs leeloo leeloo_calibration leeloo_gazebo
source install/setup.bash

ros2 launch leeloo bringup_leeloo.launch.py sim:=true enable_calibration:=false
```

`sim:=true` (défaut `false`) débranche le matériel et lance ce paquet à la place. Le reste
de la pile — curobo, segmentation, TF, exécution — est identique au robot réel.

### Matériel simulé seul (debug isolé)

```bash
ros2 launch leeloo_gazebo leeloo_gazebo.launch.py
```

En mode `standalone` (défaut), `leeloo_gazebo` lance aussi le vrai nœud
`ridgeback_control` de `leeloo`, de sorte que `/cmd_vel` fonctionne de bout en bout sans
le bringup complet.

### Tests de timing MPC

```bash
ros2 launch leeloo bringup_leeloo.launch.py sim:=true enable_calibration:=false \
  enabled_cameras:=none
```

Les caméras sont le coût dominant de la scène. Voir [Fidélité temporelle](#fidélité-temporelle--lire-avant-de-conclure-sur-le-mpc) :
c'est une **précaution obligatoire**, pas une optimisation.

> `enabled_cameras:=''` est rejeté par launch (« malformed launch argument »). Utiliser
> `none`.

---

## Ce qui est substitué

```
                     sim:=true                              sim:=false (défaut)
─────────────────────────────────────────────────────────────────────────────────────
 BRAS    curobo_ros (JointSpeedStrategy, planner_type: mpc)          ← identique
           └→ /leeloo/execute_trajectory      (JointTrajectory)
         leeloo/execute_trajectory                                   ← identique
           └→ /dsr01/dsr_controller2/speedj_rt_stream  (SpeedjRtStream, deg/s)
                        │                                       │
                   dsr_sim_bridge                          dsr_controller2
                   → arm_velocity_controller               → DRFL speedj_rt()
                   → gz_ros2_control (velocity)            → matériel
         retour   /joint_states → dsr_sim_bridge           driver Doosan
                   → /dsr01/joint_states                     → /dsr01/joint_states

 BASE    /cmd_vel → leeloo/ridgeback_control                         ← identique
           └→ /r100_0597/joy_teleop/cmd_vel   (TwistStamped)
                        │                                       │
                  base_sim_bridge                        firmware Clearpath
                  → gz MecanumDrive                      (mixage roues + watchdog)
         odométrie  /r100_0597/odom, /r100_0597/tf        (mêmes topics)

 PINCE   grasp → /robotiq_gripper_controller/gripper_cmd  (ParallelGripperCommand)
                  même nom ET même type de contrôleur des deux côtés

 CAMÉRAS  capteurs gz depth_camera                        drivers k4a / depthai
           → /depth_to_rgb/{image_raw,camera_info}
           → /oak/stereo/{image_raw,camera_info}
                        └────────── identique ──────────┘
                  robot_segmentation → /masked_depth_image{,_oak_wrist}
                    → carte voxel / ESDF curobo
```

`sim:=true` débranche : `dsr_bringup2`, le `ros2_control_node` de la pince réelle, les deux
drivers caméra, `kinect_tf_computation_node`, `leeloo_viz_state_publisher` (remplacé par le
RSP de ce paquet, qui publie une description *surensemble*) et la chaîne de détection de
marqueurs.

---

## Nœuds fournis

| Nœud | Rôle |
|---|---|
| `dsr_sim_bridge` | Tient la place de `dsr_controller2`. Consomme `/dsr01/dsr_controller2/speedj_rt_stream`, convertit `vel` de **deg/s → rad/s** vers `arm_velocity_controller`, et republie les 6 joints du bras sur `/dsr01/joint_states` (ce que fait aussi le vrai driver). Watchdog : zéro vitesse après `3 × command_period` sans commande. |
| `base_sim_bridge` | Tient la place du firmware Clearpath. Consomme la vraie sortie de `ridgeback_control` (`TwistStamped`), sature les vitesses, republie en `Twist` vers `MecanumDrive`. Watchdog 0.25 s. |

Les deux watchdogs ne sont pas cosmétiques : le plugin `MecanumDrive` et
`gz_ros2_control` conservent **indéfiniment** la dernière consigne, alors que le firmware
réel et le DRCF s'arrêtent d'eux-mêmes. Sans eux, un robot simulé continue de bouger après
la mort de son commandeur.

## Contrôleurs `ros2_control`

Définis dans [`config/leeloo_controllers.yaml`](config/leeloo_controllers.yaml), portés par
`gz_ros2_control/GazeboSimSystem` à 100 Hz.

| Contrôleur | Type | État |
|---|---|---|
| `joint_state_broadcaster` | `joint_state_broadcaster/JointStateBroadcaster` | active |
| `arm_velocity_controller` | `forward_command_controller/ForwardCommandController` (`velocity`) | **active** — chemin de production curobo/MPC |
| `arm_controller` | `joint_trajectory_controller/JointTrajectoryController` (`position`) | **inactive** — tests ponctuels sans curobo |
| `robotiq_gripper_controller` | `parallel_gripper_action_controller/GripperActionController` | active |

`arm_velocity_controller` et `arm_controller` revendiquent les mêmes joints : **un seul peut
être actif**. Pour basculer en position :

```bash
ros2 control switch_controllers --deactivate arm_velocity_controller \
                                --activate arm_controller
```

La base **n'est pas** dans `ros2_control` : elle est pilotée par le plugin cinématique
`gz::sim::systems::MecanumDrive`, déclaré dans la description.

---

## Arguments de lancement

| Argument | Défaut | Description |
|---|---|---|
| `world` | `worlds/leeloo_world.sdf` | Monde SDF. |
| `spawn_z` | `0.03` | Hauteur d'apparition. La hauteur de repos calculée est 0.0259 m (rayon roue 0.0759 − offset z 0.05) ; la marge évite chute et interpénétration. |
| `standalone` | `true` | Lance aussi `ridgeback_control` pour que `/cmd_vel` marche seul. Mis à `false` par le bringup, qui le lance déjà. |
| `enabled_cameras` | `kinect,oak` | Mêmes valeurs que `cameras.launch.py`. `none` pour aucune. |
| `use_sim_time` | `true` | Voir la mise en garde ci-dessous. |
| `tf_remap` / `tf_static_remap` | `/leeloo/tf` / `/leeloo/tf_static` | Alignés sur le robot réel. |
| `kinect_xyz` / `kinect_rpy` | `0 0 0.6` / `0 0.1 0.449` | Pose **nominale** de la Kinect sur `camera_mount` — un substitut au résultat de calibration main-œil, pas une mesure. Visée : ~(0.9, 0, 0.9) dans `base_link`. |
| `oak_xyz` / `oak_rpy` | cf. `cameras.launch.py` | Offsets eye-in-hand sur `dsr01/link_6`, mêmes valeurs que celles passées au driver depthai. |

---

## Fidélité temporelle — lire avant de conclure sur le MPC

**Toute la chaîne d'exécution est cadencée sur l'horloge murale**, pas sur le temps
simulé : la boucle servo MPC de curobo utilise `time.time()`, `joint_speed_strategy`
`time.monotonic()`, et `execute_trajectory.cpp` un `create_wall_timer`. `use_sim_time:=true`
ne change **rien** à cela — il n'est là que pour rendre cohérents les timestamps TF et
images avec `/clock` (sans quoi tout lookup TF échoue, le temps sim partant de ~0 face à un
temps mur de ~1.7e9).

Conséquence : la fidélité temporelle exige **RTF ≈ 1**. Mesuré sur ce Jetson (aarch64/Tegra) :

| Configuration | RTF |
|---|---|
| 2 caméras 1280×720, sans curobo | **1.00** |
| 2 caméras + curobo au repos | 0.80 – 0.88 |
| 2 caméras + MPC en exécution | **0.50 – 1.03**, chutes à ~0.5 pendant les résolutions |

La validation **fonctionnelle** est donc solide, mais les chiffres de latence et de
convergence MPC **ne sont pas transposables au robot réel**. Pour ces mesures : lancer avec
`enabled_cameras:=none`, ou utiliser l'image Docker x86 (`curobo_ros/docker/start_docker_x86.sh`).

> La résolution des caméras n'est **pas** un levier libre : 1280×720 est imposé par
> `mapper_image_width` / `mapper_image_height` dans `leeloo/launch/control.launch.py`.

### Le démarrage est lent, et ça ressemble à une panne

Gazebo ne crée le topic racine `/clock` qu'après avoir constaté l'absence de publisher
(« Found no publishers on /clock, adding root clock topic »), et `ros_gz_bridge` doit
ensuite se reconnecter. Pendant **~20–30 s**, le `/clock` ROS ne transporte rien, donc tous
les nœuds `use_sim_time` restent à t=0 :

- `/joint_states` est estampillé `sec=0`
- `robot_state_publisher` ne publie **aucune** TF dynamique
- `controller_manager` émet quelques centaines de « No clock received, using time argument
  instead! »

Tout se rétablit seul. **Ne pas débugger la TF ou l'horloge avant ~60 s d'uptime** — mesurer
à nouveau d'abord.

---

## Chirurgie XML sur `leeloo.urdf`

`leeloo.urdf` est un fichier plat déjà expansé par xacro, sans macro à paramétrer, et il
embarque un bloc `<ros2_control>` pour le **vrai** driver Robotiq
(`robotiq_driver/RobotiqGripperHardwareInterface`, `COM_port /dev/ttyUSB0`) qui ne peut pas
tourner en simulation. Il ne contient par ailleurs **aucun lien caméra** : la pose de la
Kinect vient de la calibration au runtime, et l'OAK-D a son propre RSP dans le driver
depthai.

`leeloo_gazebo.launch.py` construit donc la description en mémoire :

1. suppression des blocs `<ros2_control>` du matériel réel ;
2. ajout des blocs `<ros2_control>` sim (`gz_ros2_control/GazeboSimSystem`) — bras avec
   interfaces `position` **et** `velocity`, pince avec ses 6 joints ;
3. ajout des plugins `<gazebo>` (`gz_ros2_control`, `MecanumDrive`) ;
4. injection des deux caméras : lien *corps* + frame *optique* + capteur `depth_camera` ;
5. amortissement du pivot de suspension `front_rocker` (déclaré `effort="0"` sans
   `<dynamics>` : il oscille librement en Gazebo).

Les fichiers sous `leeloo/urdf/` ne sont jamais touchés.

### Repères caméra

Chaque caméra reçoit **deux** liens : un lien *corps* (convention ROS, X vers l'avant) qui
porte le capteur — gz rend une caméra selon le +X de son lien — et un enfant *optique*
(convention OpenCV, Z vers l'avant) qui porte le nom attendu par `cameras.yaml` :

- `camera_mount` → `kinect_camera_body` → **`rgb_camera_link`**
- `dsr01/link_6` → `oak_camera_body` → **`oak_left_camera_optical_frame`**

Ces poses restent en repère optique OpenCV : **ne pas** les convertir en REP-103 dans ce
pipeline.

---

## Vérification

Prérequis à chaque étape — les processus fantômes faussent les résultats, voir
[Dépannage](#dépannage).

```bash
# 1. Contrôleurs
ros2 control list_controllers
#   arm_velocity_controller … active   |  arm_controller … inactive
#   robotiq_gripper_controller … active |  joint_state_broadcaster … active

# 2. Aucun mesh manquant (surveiller GZ_SIM_RESOURCE_PATH)
#    → 0 occurrence de "Failed to load mesh" dans le log

# 3. RTF
gz topic -e -t /world/leeloo_world/stats -n 3 | grep real_time_factor

# 4. Mimic de la pince — les SIX joints doivent bouger, pas seulement le knuckle gauche
ros2 action send_goal /robotiq_gripper_controller/gripper_cmd \
  control_msgs/action/ParallelGripperCommand \
  "{command: {name: ['robotiq_85_left_knuckle_joint'], position: [0.01]}}"
ros2 topic echo /joint_states --once

# 5. Base holonome + watchdog
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {y: 0.3}}"
ros2 topic echo /r100_0597/odom     # y augmente, x reste ~0
#   puis CTRL-C : la base doit S'ARRÊTER (watchdog)

# 6. Caméras → ESDF
ros2 topic hz /depth_to_rgb/image_raw            # ~5 Hz
ros2 topic echo /depth_to_rgb/camera_info --once # doit arriver (cf. piège ci-dessous)
ros2 topic hz /curobo_trajectory_planner/voxel_grid_sparse
```

### Commander le bras via curobo

Publier une `Pose` sur `/curobo_trajectory_planner/mpc_goal` **ne suffit pas** : le callback
ne dépose la cible que pour la boucle servo, et cette boucle **ne tourne que pendant
l'action** `/curobo_trajectory_planner/execute_trajectory` (`curobo_msgs/action/SendTrajectory`).
Il faut envoyer le goal d'action avec `target_pose` renseigné. À savoir aussi : les services
`fk` / `ik` renvoient du vide jusqu'à l'appel de `warmup_fk` / `warmup_ik`.

Résultats obtenus lors de la validation (bringup complet, `sim:=true`) : curobo publie sur
`/leeloo/execute_trajectory` à ~2.2 Hz, `execute_trajectory` streame `speedj_rt_stream` à
**33.3 Hz** (= 1/`command_period` = 1/0.03) avec **0** rejet de barrière dt, le bras suit, et
l'ESDF alimenté par les caméras simulées atteint ~126 600 voxels occupés.

---

## Limites assumées

- **`SpeedjRtStream.acc` et `.time` sont ignorés.** Sur matériel, le DRCF s'en sert pour
  lisser entre deux consignes de vitesse ; `gz_ros2_control` applique la vitesse
  directement. Le mouvement simulé est donc un peu plus agressif aux frontières de lots MPC.
  Les clamps d'accélération en amont (`max_joint_accel_dps` de curobo,
  `min/max_acceleration_dps2` d'`execute_trajectory`) s'appliquent toujours : rien
  d'illégal n'est masqué, c'est le lissage du DRCF qui n'est pas reproduit.
- **Aucune alarme Doosan**, aucun code d'erreur, aucun arrêt de sécurité.
- **`MecanumDrive` est cinématique** : il impose les vitesses des 4 roues. Pas de glissement,
  pas de perte d'adhérence, pas d'effet de charge. Les roues de `leeloo.urdf` sont de
  simples cylindres sans géométrie de rouleaux mecanum — c'est structurel, pas un réglage.
  Utile pour la logique de navigation, inutilisable pour du dimensionnement dynamique.
- **Contacts de préhension peu fidèles.** Même mimic corrigés, dartsim + collisions maillées
  de la Robotiq ne donnent pas une saisie crédible. La sim valide la *logique* de `grasp`
  (séquence, masquage, action pince), pas la prise elle-même.
- **Pas de flux RGB simulé** → la calibration main-œil et la détection de marqueurs sont hors
  périmètre (la chaîne est coupée automatiquement en `sim:=true`).
- **Couverture de profondeur légèrement optimiste** pour la Kinect : le capteur simulé rend
  de la profondeur valide sur tout le FOV couleur, là où le FOV de profondeur réel
  (NFOV_UNBINNED, 75×65°) est plus étroit.
- **Pose Kinect nominale**, à ajuster : c'est un substitut au résultat de calibration.

---

## Dépannage

### Les joints mimic de la pince ne suivent pas

Ils sont appliqués par **`ros2_control`**, pas par Gazebo. Vérifié sur cette installation
(gz-sim 8.11 / gz-physics 7.6 / sdformat 14.9) : la conversion URDF→SDF émet bien
`<axis><mimic>`, et gz-sim demande bien `gz::physics::SetMimicConstraintFeature` — mais seul
le moteur **bullet-featherstone** l'implémente, et le moteur par défaut est **dartsim**, qui
ne l'a pas. La contrainte est donc silencieusement ignorée.

D'où la règle : **tous** les joints mimic doivent figurer dans le bloc `<ros2_control>`,
**sans** `<command_interface>` (`ros2_control` refuse une interface de commande sur un joint
mimic activé), avec la commande sur le seul joint menant. Ne **pas** basculer le monde sur
bullet-featherstone pour ça : cela changerait la dynamique de tout le robot, `gz_ros2_control`
et `MecanumDrive` compris.

### Un topic gz semble branché mais rien n'arrive

Deux pièges silencieux, tous deux sans le moindre message d'erreur :

1. **Les `<topic>` de `MecanumDrive` ne sont pas namespacés par modèle.** Un nom relatif
   (`cmd_vel`) devient le topic gz **global** `/cmd_vel`, et non `/model/leeloo/cmd_vel`
   comme le ferait le défaut du plugin DiffDrive.
2. **`camera_info` : gz *remplace* le dernier segment de `<topic>`, il ne l'ajoute pas.**
   `<topic>kinect/depth</topic>` publie l'image sur `/kinect/depth` et l'info sur
   **`/kinect/camera_info`** — *pas* `/kinect/depth/camera_info`. Bridger le nom concaténé
   crée un abonné sans publisher : `camera_info` n'arrive jamais, curobo ne peut pas
   dé-projeter la profondeur, et **l'ESDF reste vide sans aucune erreur**.

Toujours diagnostiquer avec `gz topic -i -t <nom>` et lire la répartition
publishers/subscribers — un topic listé par `gz topic -l` peut n'exister que parce que
quelque chose y **souscrit**.

À noter : `gz_frame_id` est la balise que gz-sensors (8.2) lit réellement pour estampiller
le `frame_id` ; `optical_frame_id` n'est qu'un élément de schéma sdformat dans cette version
et rien ne l'exploite au runtime. Les deux sont renseignées, et sdformat avertit que
`gz_frame_id` n'est « pas défini dans SDF » — ce pass-through est précisément ce qui le fait
fonctionner.

### Tous les meshes échouent à charger

La conversion URDF→SDF réécrit chaque URI `package://<pkg>/…` en `model://<pkg>/…`
**verbatim**, sans résolution ament. Gazebo cherche alors un sous-répertoire `<pkg>` dans
`GZ_SIM_RESOURCE_PATH`. Ni `leeloo` ni `robotiq_description` n'exportent
`<gazebo_ros gazebo_model_path>` (ce ne sont pas des paquets Gazebo). Le launch file règle
ça seul via `SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", …)`. **Tout nouveau paquet
porteur de meshes ajouté à la scène doit y être ajouté aussi.**

### « Controller already loaded » / relance qui se comporte bizarrement

Des processus de la session précédente survivent. `pkill -f` n'est **pas** fiable ici : il
signale souvent un succès en laissant l'arbre vivant, et la relance entre alors en collision
avec un `controller_manager` orphelin sur un graphe ROS non namespacé.

```bash
ps -eo pid,cmd | grep -iE "gz sim|ros_gz|parameter_bridge|robot_state_publisher|\
controller_manager|dsr_sim|base_sim|ridgeback|curobo|markertracker|relay|spawner|\
execute_trajectory|robot_segmentation" | grep -v grep
# puis kill -9 sur les PID listés explicitement, et re-vérifier que le grep est vide
```

Le motif doit couvrir **tous** les nœuds du lancement, pas seulement les évidents. Cas vécu :
le motif omettait `execute_trajectory`, ce nœud a `respawn=True`, et un survivant a doublé
`speedj_rt_stream` à 66.7 Hz (2 × 33.3) — ce qui ressemblait à un bug de cadence dans le
code neuf.

### `libEGL warning: egl: failed to create dri2 screen`

Le launch file force `__GLX_VENDOR_LIBRARY_NAME=nvidia` (même raison que `cameras.launch.py`
pour le depth engine k4a : GLVND annonce sinon Mesa/llvmpipe, en logiciel). L'avertissement
peut subsister sans conséquence — c'est le RTF qu'il faut regarder, pas ce message.

---

## Fichiers

```
leeloo_gazebo/
├── launch/leeloo_gazebo.launch.py     # chirurgie XML + orchestration (docstring détaillé)
├── config/leeloo_controllers.yaml     # contrôleurs ros2_control
├── worlds/leeloo_world.sdf            # monde + obstacles statiques pour l'ESDF
└── leeloo_gazebo/
    ├── dsr_sim_bridge.py              # tient la place de dsr_controller2
    └── base_sim_bridge.py             # tient la place du firmware Clearpath
```

Le monde contient trois obstacles statiques (table, pilier, boîte) placés dans l'espace de
travail du bras et dans le champ nominal de la Kinect : sans eux, l'ESDF reste vide et
l'évitement d'obstacles n'est pas observable.

## Voir aussi

- Docstring de [`launch/leeloo_gazebo.launch.py`](launch/leeloo_gazebo.launch.py) — le
  *pourquoi* de chaque choix, au plus près du code.
- `CLAUDE.md` à la racine de `src/` — architecture d'ensemble de Leeloo et de `curobo_ros`.
- `curobo_ros/docs/concepts/ros-interfaces.md` — inventaire exhaustif des interfaces curobo.
