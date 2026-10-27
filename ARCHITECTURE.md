# Robocar ROS 2 : architecture

Jetson Nano (hôte Ubuntu 18.04), ROS 2 Jazzy dans Docker (ubuntu 24.04).\
**Version wesc** : la brique `control` passe par nos bibliothèques (`wesc` pour le VESC, sur l'UART `/dev/ttyTHS1` ; `robocar_teleop` pour la manette) au lieu de `vesc_driver`, `ackermann_to_vesc`, `joy` et `joy_teleop` (F1TENTH). Conception et plan : `docs/WESC_JAZZY.md`. **Cible, pas encore écrite** : les paquets `wesc`, `robocar_vesc`, `robocar_teleop` et `robocar_msgs` n'ont qu'un `README.md`.\
Une brique = un service `docker compose`.\
Démarrage : `README.md` (étudiants), `BOOTSTRAP.md` (préparer une voiture). Détails et TODO : `docs/ROBOCAR_TO_DO_ROS2.md`. Journal de mise en route : `A_FAIRE_SUR_LA_JETSON.md`.\
État de la version F1TENTH au 2026-09-25 : `control` (VESC, F710) et `lidar` validés sur la voiture, roues en l'air, sécurité comprise. `camera_3d_oak` n'est pas encore testé, et la calibration au sol reste à faire.

## Briques

| Service | Profil | Nœuds | Paquet |
|---|---|---|---|
| `control` | `control` | `robocar_teleop`, `ackermann_mux`, `robocar_vesc`, `vesc_to_odom_node`, `robot_state_publisher` | `robocar_teleop`, `robocar_vesc` (+ `wesc`), `third_party/` (mux, `vesc_msgs`, `vesc_to_odom`) |
| `lidar` | `lidar` | `ldlidar` | `third_party/ldlidar_stl_ros2` |
| `camera_3d_oak` | `camera_3d_oak` | `oak` (driver Luxonis) | apt `depthai_ros_driver` |
| `camera` | `camera` | `camera_node` (générique, vide, à compléter) | `robocar_camera` |
| `gps` | `gps` | *(à faire, phase 7)* | — |
| `gpu` | `gpu` | `gpu_server` hors ROS + `gpu_bridge` *(à faire, phase 8)* | — |

> **Service** : le nom du conteneur dans `docker/compose.yaml`.\
> **Profil** : l'option pour l'activer au lancement. Aucun service ne démarre sans son profil, et rien ne se relance tout seul (`restart: "no"`).\
> *Exemple : `docker compose -f docker/compose.yaml --profile lidar up -d` lance `lidar` seul ; ajouter `--profile control` pour conduire.*

## Graphe des nœuds

```mermaid
flowchart LR
  F710[(F710)] -->|"/dev/input (libevdev)"| JT[robocar_teleop]
  JT -->|"/teleop · prio 100, 50 Hz"| MUX[ackermann_mux]
  JT -->|"/drive_enable · lock, RB"| MUX
  JT -->|/teleop/state| OUT
  USER["nœud étudiant"] -->|"/drive · prio 10"| MUX
  MUX -->|/ackermann_cmd| VD["robocar_vesc (wesc)<br/>+ watchdog 0,1 s"]
  VD <-->|"/dev/vesc → ttyTHS1, UART 115200"| VESC[(VESC)]
  VD -->|"/sensors/core<br/>/sensors/servo_position_command"| V2O[vesc_to_odom]
  V2O -->|"/odom · TF odom→base_link"| OUT(( ))
  RSP[robot_state_publisher] -->|/tf_static| OUT

  LIDAR[(LiDAR)] -->|/dev/lidar| LD[ldlidar] -->|/scan| OUT
  OAKD[(OAK-D Lite)] --> OAK[oak] -->|"/oak/rgb/* · /oak/stereo/*"| OUT
  CAMX[(webcam, smartphone...)] --> CAM[camera_node] -->|/camera/*| OUT
```

## Chaîne de sécurité

```mermaid
flowchart LR
  B{Manette} -->|aucun bouton| Z["/teleop = 0 (prio 100)"]
  B -->|LB| T["/teleop = sticks (prio 100)"]
  B -->|RB| U["lock ouvert : /drive passe"]
  B -->|"autre bouton, ou manette absente > 0,1 s"| L["lock fermé : /drive masqué"]
  L --> M["mux muet"] -->|0,1 s| W["watchdog : vitesse 0"]
```

Mesures faites avec la version F1TENTH (`joy_teleop`, `ackermann_to_vesc`, `vesc_driver`). **À refaire** avec `robocar_teleop` et `robocar_vesc` (exigences : `docs/WESC_JAZZY.md` §6).

| Panne | Arrêt mesuré (PC) | Arrêt mesuré (Jetson) | Voiture, roues en l'air |
|---|---|---|---|
| RB relâché | ~2 ms | 4 ms | 6 ms |
| Autre bouton que RB | vitesse 0 | vitesse 0 | vitesse 0 |
| Manette morte (`joy` ou `joy_teleop` ; ici `robocar_teleop`) | ~200 ms | 189 ms | pas testé |
| `/drive` coupé | ~90 ms | 85 ms | 57 ms |
| Mux planté | ~115 ms | 139 ms | — |
| `ackermann_to_vesc` ou `vesc_driver` planté (ici `robocar_vesc`) | — | — | timeout du VESC, 500 ms (pas testé) |

## Topics

| Topic | Type | Publié par |
|---|---|---|
| `/drive` | `ackermann_msgs/AckermannDriveStamped` | nœud étudiant (> 10 Hz, RB maintenu) |
| `/teleop` | `ackermann_msgs/AckermannDriveStamped` | `robocar_teleop` (LB) |
| `/drive_enable` | `std_msgs/Bool` | `robocar_teleop` (RB) |
| `/teleop/state` | `robocar_msgs/TeleopState` | `robocar_teleop` |
| `/ackermann_cmd` | `ackermann_msgs/AckermannDriveStamped` | `ackermann_mux` |
| `/odom` | `nav_msgs/Odometry` | `vesc_to_odom_node` |
| `/sensors/core` | `vesc_msgs/VescStateStamped` | `robocar_vesc` |
| `/sensors/servo_position_command` | `std_msgs/Float64` | `robocar_vesc` |
| `/scan` | `sensor_msgs/LaserScan` | `ldlidar` |
| `/oak/rgb/*`, `/oak/stereo/*` | `sensor_msgs/Image` | `oak` |
| `/camera/*` | `sensor_msgs/Image` ou `CompressedImage` | `camera_node` |

## Arbre TF

```mermaid
flowchart LR
  map -->|slam_toolbox| odom -->|vesc_to_odom| base_link
  base_link --> laser
  base_link --> oakf[oak-d-base-frame] --> oakr[frames depthai]
  base_link --> camera_link
  base_link --> gps
```

`base_link` = centre de l'essieu arrière (REP-103 / REP-105).

## Dépôt

```
Robocar_ROS2/
├── docker/        Dockerfile, entrypoint.sh, compose.yaml
├── host/          udev, setup_host.sh (Jetson, hors conteneur) ; vesc/ : config VESC Tool
├── ros2_ws/src/
│   ├── wesc/                 protocole VESC en C (bibliothèque, sans ROS)
│   ├── robocar_vesc/         nœud ackermann_cmd → wesc → VESC, télémétrie → /sensors/*
│   ├── robocar_teleop/       nœud manette (libevdev, logique de soukoukou)
│   ├── robocar_msgs/         TeleopState
│   ├── third_party/          ackermann_mux, vesc (vesc_msgs + vesc_to_odom seulement), ldlidar_stl_ros2 (+ VENDORED.md) ; vesc_driver et teleop_tools plus compilés
│   ├── robocar_description/  URDF
│   ├── robocar_bringup/      launch/ + config/, un par brique
│   └── robocar_camera/       nœud caméra générique
├── tests/         test_safety_chain.py
├── pc/slam/       guide SLAM (à faire par les étudiants) + config de départ
├── docs/          ROBOCAR_TO_DO_ROS2.md, WESC_JAZZY.md (version wesc)
├── README.md      démarrage étudiants
└── BOOTSTRAP.md   préparer une voiture
```
