# Fonctionnement de la version wesc dans Docker (WIP)

Explication de **comment tourne** la version wesc : l'image, les conteneurs, ce qui se passe dans chacun, et comment ils parlent entre eux et au matériel.

> **🚧 WIP (2026-10-29)** : décrit le code du commit `87fa881` (branche `wesc`). Testé sur PC seulement, pas encore sur la voiture.

| Document | Pour quoi |
|---|---|
| `docs/WESC_JAZZY.md` | Les décisions de conception et le plan |
| `docs/WESC_DOCKER.md` | Le mode d'emploi : installer, construire, lancer, tester |
| **Ce document** | Comprendre comment ça marche |
| `ros2_ws/src/<paquet>/README.md` | Le détail de chaque paquet (paramètres, conversions) |

Les schémas sont en Mermaid : GitHub les affiche directement, VS Code aussi avec l'extension « Markdown Preview Mermaid Support ».

---

## 1. Vue d'ensemble

Une seule **image** (`robocar:jazzy`) contient tout le code. `docker compose` en lance **un conteneur par brique** : `control`, `lidar`, `camera_3d_oak`, `camera`. Chaque brique a son profil et ne démarre que si on la demande.

```mermaid
flowchart TB
  subgraph PC["PC (Ubuntu 24.04, ROS 2 Jazzy natif)"]
    RVIZ[RViz2]
    STU["nœud étudiant<br/>publie /drive"]
    SLAM[slam_toolbox]
  end

  subgraph JETSON["Jetson Nano (L4T R32.7.1, Ubuntu 18.04, Docker 20.10)"]
    direction TB
    subgraph HOST["Hôte"]
      UDEV["udev<br/>99-robocar.rules"]
      DEV["/dev/vesc → ttyTHS1<br/>/dev/lidar → ttyUSB*<br/>/dev/input/event*<br/>/dev/bus/usb"]
      UDEV --> DEV
    end
    subgraph DOCKER["docker compose · image robocar:jazzy · network_mode: host"]
      C1["control<br/>(profil control)"]
      C2["lidar<br/>(profil lidar)"]
      C3["camera_3d_oak<br/>(profil camera_3d_oak)"]
      C4["camera<br/>(profil camera)"]
    end
    DEV -->|/dev/vesc, /dev/input| C1
    DEV -->|/dev/lidar| C2
    DEV -->|/dev/bus/usb| C3
  end

  VESC[(VESC<br/>moteur + servo)] <-->|UART 115200| DEV
  F710[(Manette F710<br/>dongle USB)] --> DEV
  LIDAR[(LiDAR LD19)] --> DEV
  OAK[(OAK-D Lite)] --> DEV

  C1 & C2 & C3 & C4 <--> BUS{{"DDS · ROS_DOMAIN_ID<br/>tous les topics ROS"}}
  BUS <-->|Wi-Fi / Ethernet| PC
```

Trois idées à retenir :

1. **Même image, plusieurs conteneurs.** On construit une fois (`docker compose build control`), et tous les services réutilisent `robocar:jazzy`. Seule la commande lancée change (`ros2 launch robocar_bringup <brique>.launch.py`).
2. **Réseau `host` et `ipc: host`.** Les conteneurs n'ont pas de réseau à eux : ils partagent celui de la Jetson. Les nœuds ROS de tous les conteneurs et du PC se trouvent donc par la découverte DDS normale, comme s'ils tournaient directement sur la machine. `ipc: host` permet la mémoire partagée entre conteneurs.
3. **Le matériel est donné au cas par cas.** Chaque conteneur ne voit que les périphériques listés pour lui dans `compose.yaml`. Seul `control` peut parler au VESC et à la manette.

---

## 2. L'image : ce qu'il y a dedans

`docker/Dockerfile`, construit **sur la Jetson** (arm64). Chaque étape est une couche ; Docker garde en cache celles qui n'ont pas changé.

```mermaid
flowchart TB
  A["<b>ros:jazzy-ros-base</b><br/>Ubuntu 24.04 + ROS 2 Jazzy (rclcpp, rclpy, DDS)<br/>image officielle, non modifiée"]
  B["<b>apt</b> (couche en cache)<br/>ackermann_msgs · libevdev-dev · pkg-config<br/>robot_state_publisher · xacro<br/>depthai_ros_driver · image_transport_plugins"]
  C["<b>COPY ros2_ws/src</b><br/>notre code + third_party"]
  D["<b>rosdep install</b><br/>rattrape les dépendances oubliées"]
  E["<b>colcon build</b> --parallel-workers 1, Release<br/>11 paquets → /ros2_ws/install"]
  F["<b>entrypoint.sh</b><br/>vérifie ROS_DOMAIN_ID, source les setup.bash"]
  A --> B --> C --> D --> E --> F

  style A fill:#e8e8e8,stroke:#888
  style B fill:#dbeafe,stroke:#3b82f6
  style C fill:#fef3c7,stroke:#d97706
  style E fill:#dcfce7,stroke:#16a34a
```

Modifier du code ne refait que les couches à partir de `COPY` : la couche `apt`, la plus longue à télécharger, reste en cache.

Les 11 paquets compilés, et le conteneur qui s'en sert :

```mermaid
flowchart LR
  subgraph NOTRE["Notre code"]
    WESC["wesc<br/>bibliothèque C"]
    RV["robocar_vesc<br/>C++"]
    RT["robocar_teleop<br/>C++"]
    RM["robocar_msgs"]
    RB["robocar_bringup<br/>launch + config"]
    RD["robocar_description<br/>URDF"]
    RC["robocar_camera<br/>Python"]
  end
  subgraph TP["third_party (code copié)"]
    MUX[ackermann_mux]
    VMSG[vesc_msgs]
    VACK["vesc_ackermann<br/>(vesc_to_odom seulement)"]
    LDL[ldlidar_stl_ros2]
  end
  subgraph IGN["Plus compilés (COLCON_IGNORE)"]
    X1[vesc_driver]
    X2["teleop_tools / joy_teleop"]
  end

  WESC -->|lié dans| RV
  VMSG --> RV
  RM --> RT
  RV & RT & MUX & VACK & RD -.->|control| RB
  LDL -.->|lidar| RB
  RC -.->|camera| RB

  style IGN fill:#fee2e2,stroke:#dc2626
```

`wesc` n'est pas un nœud : c'est une bibliothèque statique (`libwesc.a`), compilée **dans** l'exécutable `robocar_vesc_node`. Elle ne dépend pas de ROS, et peut toujours se compiler seule avec son `Makefile` (comme dans `le_loto`).

---

## 3. Démarrage d'un conteneur

Tous les conteneurs démarrent de la même façon :

```mermaid
sequenceDiagram
  participant U as Utilisateur
  participant DC as docker compose
  participant E as entrypoint.sh
  participant L as ros2 launch

  U->>DC: docker compose --profile control up -d
  DC->>DC: lit .env (ROS_DOMAIN_ID, COMPOSE_FILE)
  DC->>DC: vérifie que /dev/vesc existe (devices:)
  Note over DC: absent → le conteneur ne démarre pas
  DC->>E: lance le conteneur
  E->>E: ROS_DOMAIN_ID entre 1 et 101 ?
  Note over E: sinon : message d'erreur et arrêt
  E->>E: source /opt/ros/jazzy + /ros2_ws/install
  E->>L: exec ros2 launch robocar_bringup control.launch.py
  L->>L: démarre chaque nœud dans son propre processus
```

`exec` remplace le shell par `ros2 launch` : c'est lui qui reçoit le `SIGTERM` de `docker compose down` et le transmet aux nœuds, qui s'arrêtent proprement (pour `robocar_vesc` : moteur à 0 et servo au neutre).

Pas de redémarrage automatique (`restart: "no"`), même au démarrage de la Jetson : un conteneur arrêté reste arrêté.

---

## 4. Conteneur `control`

C'est le seul qui touche aux moteurs. Il lance **5 nœuds**, chacun dans **son propre processus** :

```mermaid
flowchart LR
  subgraph HOSTDEV["Périphériques de l'hôte"]
    IN["/dev/input<br/>(volume + cgroup c 13:*)"]
    TTY["/dev/vesc → ttyTHS1<br/>(devices)"]
  end

  subgraph CONTROL["Conteneur control · control.launch.py"]
    direction LR
    TEL["<b>robocar_teleop</b><br/>libevdev<br/>teleop.yaml"]
    MUX["<b>ackermann_mux</b><br/>priorités + lock<br/>mux.yaml"]
    RV["<b>robocar_vesc</b><br/>wesc + watchdog<br/>vesc.yaml"]
    V2O["<b>vesc_to_odom</b><br/>vesc.yaml"]
    RSP["<b>robot_state_publisher</b><br/>URDF"]

    TEL -->|"/teleop · 50 Hz · prio 100<br/>(muet si RB seul)"| MUX
    TEL -->|"/drive_enable · 50 Hz tant que RB"| MUX
    MUX -->|/ackermann_cmd| RV
    RV -->|"/sensors/core · 50 Hz"| V2O
    RV -->|/sensors/servo_position_command| V2O
  end

  IN --> TEL
  RV <--> TTY

  STU["nœud étudiant<br/>(PC ou autre conteneur)"] -->|"/drive · > 10 Hz · prio 10"| MUX
  TEL -->|"/teleop/state · 20 Hz"| OUT((sorties))
  V2O -->|"/odom + TF odom → base_link"| OUT
  RSP -->|"/tf_static · base_link → laser, oak, camera_link, gps"| OUT
```

### 4.1 `robocar_teleop` : de la manette à `/teleop`

Lit la F710 **directement dans le fichier** `/dev/input/by-id/*F710*-event-joystick`, avec libevdev. Pas de SDL, pas de nœud `joy`, pas de topic `/joy` : un seul processus au lieu de deux.

```mermaid
flowchart TB
  subgraph T["robocar_teleop · un seul fil, 4 timers"]
    R["<b>reconnect</b> · toutes les 2 s<br/>manette absente ? → glob by-id, open, libevdev_new_from_fd"]
    RAMP["<b>rampe</b> · 20 Hz (50 ms, comme soukoukou)<br/>lit les événements · drive_ramp.hpp<br/>RT → avance (expo RT²)<br/>LT → freine, puis recule à l'arrêt<br/>changement de sens : pause 0,2 s"]
    PUB["<b>publication</b> · 50 Hz<br/>lit les événements<br/>/teleop : rampe × max_speed si LB, 0 sinon<br/>rien si RB et que la manette ne conduit pas<br/>/drive_enable = false si RB"]
    ST["<b>état</b> · 20 Hz<br/>/teleop/state (collecte de données)"]
  end
  EV[("/dev/input/eventN")] -->|"libevdev_next_event<br/>non bloquant"| RAMP & PUB
  R -.->|ouvre| EV
```

Mapping jeu de voiture : RT avance, LT freine puis recule une fois arrêté. La logique est dans `drive_ramp.hpp`, testée sans manette :

```mermaid
stateDiagram-v2
  direction LR
  [*] --> Arret
  Arret --> Avant: RT
  Arret --> Arriere: LT
  Avant --> Avant: RT (rampe)
  Avant --> Freinage: LT
  Freinage --> Pause: vitesse 0
  Freinage --> Avant: LT relâché, RT
  Avant --> Arret: rien (décélère)
  Pause --> Arriere: LT toujours, après 0,2 s
  Pause --> Arret: LT relâché
  Arriere --> Arriere: LT (rampe)
  Arriere --> FreinageAr: RT
  FreinageAr --> Pause2: vitesse 0
  Pause2 --> Avant: RT, après 0,2 s
  Arriere --> Arret: rien (décélère)
  Pause: Pause 0,2 s
  Pause2: Pause 0,2 s
  Arret: Arrêt
  Arriere: Arrière
  FreinageAr: Freinage (RT)
```

Avec `require_deadman: true` (défaut), tout ceci n'agit que LB maintenu ; relâcher LB ramène à « Arrêt » d'un coup. Avec `false`, RT et LT suffisent, comme dans un jeu.

- **Tout dans un fil.** Les 4 timers tournent dans le même exécuteur ROS : jamais deux callbacks en même temps, donc pas de verrou.
- **libevdev garde l'état.** Le nœud vide la file d'événements, puis demande à libevdev la valeur courante de chaque bouton et axe. Si des événements sont perdus (`SYN_DROPPED`), libevdev resynchronise.
- **Manette perdue** (`ENODEV`) : le fichier est fermé, **plus rien n'est publié**, et le timer `reconnect` la cherche toutes les 2 s. Le volume `/dev/input` et la règle `c 13:* rmw` permettent de rouvrir le nouveau `eventN` après un rebranchement, sans redémarrer le conteneur.

### 4.2 `ackermann_mux` : qui a le droit de conduire

Inchangé par rapport à la version F1TENTH (`mux.yaml`). Il choisit la source de plus haute priorité encore active, et la recopie sur `/ackermann_cmd` :

```mermaid
flowchart LR
  T["/teleop · prio 100<br/>timeout 0,2 s"] --> M{ackermann_mux}
  D["/drive · prio 10<br/>timeout 0,2 s"] --> M
  L["lock drive_enable · prio 100<br/>timeout 0,1 s"] -.->|"verrouillé : masque<br/>tout ce qui est < 100"| M
  M --> A["/ackermann_cmd"]
```

| Boutons | `/teleop` (100) | lock `drive_enable` | Ce qui sort sur `/ackermann_cmd` |
|---|---|---|---|
| Rien | publié, vitesse 0 | verrouillé | `/teleop` à 0 : arrêt, même si `/drive` est publié |
| LB (+ RT / LT) | publié, rampe | verrouillé | `/teleop` : conduite manuelle |
| RB | **muet** | ouvert | `/drive` (après expiration de `/teleop`, 0,2 s) : le nœud étudiant conduit |
| RB + LB | publié, rampe | ouvert | `/teleop` (100 > 10) : la manette reprend la main |
| Manette perdue | muet | verrouillé (expiré) | rien → watchdog de `robocar_vesc` |

Avec `require_deadman: false`, « LB » se lit « une gâchette enfoncée, ou la voiture pas encore arrêtée ».

Le point clé : quand RB est maintenu seul, `robocar_teleop` **arrête de publier `/teleop`**. Sinon `/teleop`, même à vitesse 0, garderait la priorité 100 et `/drive` ne passerait jamais. C'est ce que faisait le profil `default` de `joy_teleop`, inactif dès qu'un bouton est maintenu.

> Le comportement exact du mux (priorités égales, lock) est celui mesuré sur la version F1TENTH (10/10 dans `A_FAIRE_SUR_LA_JETSON.md`). Il est à remesurer avec `robocar_teleop` (`docs/WESC_DOCKER.md` §6).

### 4.3 `robocar_vesc` : de `/ackermann_cmd` au VESC

Le seul processus qui ouvre `/dev/vesc`. Tout se passe dans **un seul timer à 50 Hz** ; le callback `/ackermann_cmd` ne fait que noter la consigne et son heure (horloge monotone).

```mermaid
flowchart TB
  START(["tick · toutes les 20 ms"]) --> OPEN{port ouvert ?}
  OPEN -->|non| TRY["wesc_open(/dev/vesc, 115200)<br/>échec → log, réessai au tick suivant"] --> FIN
  OPEN -->|oui| FRESH{"consigne reçue<br/>il y a moins de 0,1 s ?"}
  FRESH -->|oui| M["moteur<br/>speed : wesc_set_rpm(gain × v)<br/>duty : wesc_set_duty(zone morte + v / max)<br/>v = 0 : wesc_set_duty(0)"]
  M --> S["servo si changé<br/>wesc_set_servo(gain × angle + offset)<br/>publie /sensors/servo_position_command"]
  S --> AL["wesc_send_alive"]
  FRESH -->|"non : WATCHDOG"| Z["wesc_set_duty(0)<br/><b>pas d'ALIVE</b>"]
  AL --> GV
  Z --> GV["wesc_request_values<br/>wesc_read_values (20 ms max)"]
  GV -->|OK| PUB["publie /sensors/core"] --> FIN
  GV -->|"timeout, CRC faux"| WARN[log] --> FIN
  GV -->|"erreur série"| ERR["duty 0, ferme le port<br/>réouverture au tick suivant"] --> FIN
  FIN([fin du tick])

  style Z fill:#fee2e2,stroke:#dc2626
  style AL fill:#dcfce7,stroke:#16a34a
```

Pourquoi « pas d'`ALIVE` » sans consigne : le VESC coupe le moteur seul s'il ne reçoit plus rien d'utile pendant **500 ms** (son propre timeout). En n'envoyant `ALIVE` que s'il a une consigne fraîche, `robocar_vesc` laisse ce dernier filet fonctionner : si le nœud se bloque ou meurt, le VESC s'arrête de lui-même. C'est la différence principale avec soukoukou, dont un processus fils envoyait `ALIVE` en continu, même si le programme principal était bloqué.

### 4.4 `wesc` : ce qui passe sur le fil

Chaque commande est une **trame** courte. Exemple pour `wesc_set_duty(fd, 0.06)` :

```mermaid
flowchart LR
  A["0x02<br/>début"] --> B["0x05<br/>longueur"] --> C["0x05<br/>SET_DUTY"] --> D["00 00 17 70<br/>6000 = 0,06 × 100000<br/>int32 gros-boutiste"] --> E["C7 24<br/>CRC16 de la charge utile"] --> F["0x03<br/>fin"]
  style C fill:#dbeafe
  style D fill:#dbeafe
```

| Fonction | Commande VESC | Charge utile |
|---|---|---|
| `wesc_send_alive` | `ALIVE` (30) | — |
| `wesc_set_duty` | `SET_DUTY` (5) | duty × 100000, int32 |
| `wesc_set_rpm` | `SET_RPM` (8) | ERPM, int32 |
| `wesc_set_servo` | `SET_SERVO_POS` (12) | position × 1000, int16 |
| `wesc_set_current`, `wesc_set_brake` | 6, 7 | ampères × 1000, int32 |
| `wesc_request_values` | `GET_VALUES` (4) | — ; réponse de 73 octets décodée par `wesc_read_values` |

À la lecture, `wesc` attend avec `poll()` jusqu'à un délai maximal, ignore les octets parasites avant un début de trame, et vérifie le CRC et l'octet de fin.

Un tick complet de `robocar_vesc`, consigne active, sur le fil (environ 30 octets dans chaque sens, sur les 230 que permet 115200 bauds en 20 ms) :

```mermaid
sequenceDiagram
  participant RV as robocar_vesc
  participant V as VESC
  RV->>V: SET_RPM (10 octets)
  RV->>V: SET_SERVO_POS (8 octets, si changé)
  RV->>V: ALIVE (6 octets)
  RV->>V: GET_VALUES (6 octets)
  V-->>RV: réponse GET_VALUES (78 octets)
  Note over RV: publie /sensors/core
```

### 4.5 `vesc_to_odom` et `robot_state_publisher`

- **`vesc_to_odom`** (F1TENTH, inchangé) : lit la vitesse mesurée (`/sensors/core`, ERPM ÷ `speed_to_erpm_gain`) et l'angle du servo (`/sensors/servo_position_command`), intègre, et publie `/odom` et la TF `odom → base_link`. Comme `robocar_vesc` publie les mêmes topics et types que `vesc_driver`, il n'a pas eu besoin d'être modifié.
- **`robot_state_publisher`** : lit l'URDF (`robocar_description`) et publie les TF fixes des capteurs.

```mermaid
flowchart TB
  O[odom] -->|"vesc_to_odom (dynamique)"| B[base_link]
  B -->|"URDF (fixe)"| L["laser<br/>x 0,25 z 0,15"]
  B --> K["oak-d-base-frame<br/>x 0,30 z 0,12"]
  B --> C["camera_link<br/>x 0,30 z 0,16"]
  B --> G["gps<br/>x 0,10 z 0,20"]
  K -->|depthai| KO["oak_* (frames de la caméra)"]
  M[map] -.->|"slam_toolbox (PC), plus tard"| O
```

### 4.6 La chaîne de sécurité de bout en bout

Ce qui arrête la voiture, selon ce qui tombe en panne :

```mermaid
flowchart LR
  P1["RB relâché"] --> MUX1["lock expire (0,1 s)<br/>/drive masqué"]
  P2["manette perdue"] --> TEL2["teleop ne publie plus<br/>/teleop et lock expirent"]
  P3["nœud étudiant planté"] --> MUX3["/drive expire (0,2 s)"]
  MUX1 & TEL2 & MUX3 --> SIL["mux : plus rien<br/>sur /ackermann_cmd"]
  P4["mux planté"] --> SIL
  SIL --> WD["watchdog robocar_vesc (0,1 s)<br/>duty 0, plus d'ALIVE"]
  P5["robocar_vesc planté<br/>ou câble UART coupé"] --> VT["timeout VESC (0,5 s)"]
  WD --> STOP([moteur arrêté])
  VT --> STOP
  WD -.->|"si le fil se bloque"| VT

  style STOP fill:#dcfce7,stroke:#16a34a
```

Délais visés : moins de 0,2 s pour les pannes de la manette, du nœud étudiant et du mux ; 0,5 s au plus si `robocar_vesc` meurt. **À mesurer sur la voiture** (`docs/WESC_DOCKER.md` §6).

---

## 5. Conteneur `lidar`

Un seul nœud, code tiers (`ldlidar_stl_ros2`), inchangé par la version wesc.

```mermaid
flowchart LR
  LD[(LD19 / STL-19P)] -->|"USB-série CP2102<br/>230400 bauds"| DEV["/dev/lidar<br/>(devices)"]
  subgraph LIDAR["Conteneur lidar · lidar.launch.py"]
    N["<b>ldlidar</b><br/>ldlidar_stl_ros2_node<br/>lidar.yaml"]
  end
  DEV --> N
  N -->|"/scan · ~10 Hz · frame laser<br/>~500 points par tour"| OUT((PC : RViz, SLAM))
```

- La TF `base_link → laser` ne vient **pas** de ce conteneur, mais de `robot_state_publisher` dans `control`. Sans `control`, prendre `laser` comme Fixed Frame dans RViz.
- Le LiDAR et le VESC sont dans **des conteneurs séparés** : on peut redémarrer l'un sans couper l'autre, et un plantage du pilote LiDAR n'atteint pas la conduite.

---

## 6. Conteneur `camera_3d_oak`

Pilote officiel Luxonis (`depthai_ros_driver`, installé par apt), inchangé.

```mermaid
flowchart LR
  OAK[(OAK-D Lite)] -->|USB| BUS["/dev/bus/usb<br/>(volume, privileged)"]
  subgraph CAM3D["Conteneur camera_3d_oak · camera_3d_oak.launch.py"]
    D["<b>oak</b><br/>depthai_ros_driver<br/>RGBD, 15 fps<br/>camera_3d_oak.yaml"]
  end
  BUS --> D
  D -->|"/oak/rgb/* · 640x360"| OUT((PC))
  D -->|"/oak/stereo/* · profondeur 480P"| OUT
  D -->|"TF oak-d-base-frame → oak_*"| OUT
```

`privileged` et tout `/dev/bus/usb` : au démarrage, la caméra change d'identifiant USB (chargeur d'amorçage, puis application). Un seul `devices:` ne suivrait pas ce changement.

---

## 7. Conteneur `camera`

Squelette d'un nœud caméra générique (`robocar_camera`, Python), **pas encore écrit** : il démarre mais ne publie rien.

```mermaid
flowchart LR
  SRC[("webcam USB<br/>ou caméra réseau")] -.->|"/dev/video0 si USB<br/>(à décommenter)"| N
  subgraph CAM["Conteneur camera · camera.launch.py"]
    N["<b>camera_node</b><br/>robocar_camera · rclpy<br/>camera.yaml"]
  end
  N -.->|"/camera/image_raw · frame camera_link<br/>(à écrire)"| OUT((PC))
```

---

## 8. Récapitulatif des conteneurs

| Conteneur | Profil | Nœuds | Accès matériel | Publie |
|---|---|---|---|---|
| `control` | `control` | `robocar_teleop`, `ackermann_mux`, `robocar_vesc`, `vesc_to_odom`, `robot_state_publisher` | `/dev/vesc` (devices), `/dev/input` (volume + cgroup `c 13:*`) | `/teleop`, `/drive_enable`, `/teleop/state`, `/ackermann_cmd`, `/sensors/*`, `/odom`, `/tf`, `/tf_static` |
| `lidar` | `lidar` | `ldlidar` | `/dev/lidar` (devices) | `/scan` |
| `camera_3d_oak` | `camera_3d_oak` | `oak` (depthai) | `/dev/bus/usb` (privileged) | `/oak/rgb/*`, `/oak/stereo/*` |
| `camera` | `camera` | `camera_node` | aucun pour l'instant | rien pour l'instant |

Communs à tous : image `robocar:jazzy`, `network_mode: host`, `ipc: host`, `restart: "no"`, variables de `.env` (`ROS_DOMAIN_ID`, `ROS_AUTOMATIC_DISCOVERY_RANGE`).

---

## 9. Pourquoi c'est plus léger que la version F1TENTH

```mermaid
flowchart LR
  subgraph AVANT["Version F1TENTH · 7 processus pour conduire"]
    direction LR
    a1[joy<br/>SDL2] -->|/joy| a2[joy_teleop<br/>Python] -->|/teleop| a3[ackermann_mux] -->|/ackermann_cmd| a4[ackermann_to_vesc] -->|"/commands/motor/*<br/>/commands/servo/*"| a5[vesc_driver] -->|USB| a6[(VESC)]
  end
  subgraph APRES["Version wesc · 3 processus pour conduire"]
    direction LR
    b1["robocar_teleop<br/>libevdev, C++"] -->|/teleop| b2[ackermann_mux] -->|/ackermann_cmd| b3["robocar_vesc<br/>wesc, C"] -->|UART| b4[(VESC)]
  end
```

| | F1TENTH | wesc |
|---|---|---|
| Sauts DDS entre la manette et le VESC | 4 | 2 |
| Lecture de la manette | SDL2, par `joy` (dépend de la base udev, absente du conteneur) | `read()` sur `/dev/input/eventN`, par libevdev |
| Protocole VESC | `vesc_driver` (C++, `serial_driver`, `asio`) | `wesc` (C, ~400 lignes, `write()` / `poll()`) |
| Liaison VESC | micro-USB (connecteur fragile) | UART du J41 |
| Dépendances apt retirées | — | `ros-jazzy-joy`, `ros-jazzy-serial-driver`, `ros-jazzy-asio-cmake-module` |

Le mux reste un saut DDS de plus : on l'a gardé pour ne pas réécrire une chaîne de sécurité déjà mesurée (`docs/WESC_JAZZY.md` §1).
