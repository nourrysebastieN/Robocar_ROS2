# Lancer la version wesc dans Docker (WIP)

Mode d'emploi de l'image `robocar:jazzy` construite avec nos paquets : `wesc` (VESC par l'UART), `robocar_vesc`, `robocar_teleop` (manette lue par libevdev). Conception : `docs/WESC_JAZZY.md`.

> **🚧 WIP (2026-10-28) : pas encore essayé sur la voiture.** Image construite et testée **sur PC seulement** (amd64, VESC simulé). Rien n'a encore tourné sur la Jetson : ce guide peut changer après le premier essai. Les lignes **« à vérifier »** n'ont pas pu être contrôlées sur la voiture.

Ce qui change pour qui connaît la version F1TENTH (`README.md`, `BOOTSTRAP.md`) :

| | Version F1TENTH | Version wesc |
|---|---|---|
| Liaison VESC | USB, `/dev/vesc → ttyACM0` | **UART du J41**, `/dev/vesc → ttyTHS1` |
| Manette | `joy` (SDL2) + `joy_teleop`, config `joy_teleop.yaml` | `robocar_teleop` (libevdev), config `teleop.yaml` |
| Accélérer | LB + stick gauche vertical | LB + **gâchette RT**, frein **LT** |
| Direction | stick droit | **stick gauche** horizontal |
| Marche arrière | stick vers le bas | bouton **Y** (bascule avant / arrière) |
| Message au démarrage | `Out-of-sync with VESC`, puis `Connected to VESC` | `/dev/vesc ouvert` |

Ce qui ne change pas : le service `control`, le mux, le lock RB sur `/drive`, `/odom`, les autres services (`lidar`, caméras), `.env`.

## 1. Préparer la Jetson (une seule fois)

Prérequis déjà faits sur `soukoukou` pour la version F1TENTH (`A_FAIRE_SUR_LA_JETSON.md`, « Déjà fait » C et D) : plugin `docker compose`, `setup_host.sh` (groupes, swap, chrony). Ne pas les refaire.

### a. Câblage UART

VESC TX → Jetson RX (J41 broche 10), VESC RX → Jetson TX (broche 8), masse (broche 6). 3,3 V des deux côtés, jamais de 5 V. **Débrancher l'USB du VESC** : un seul lien à la fois. Rien à changer dans VESC Tool (UART déjà actif à 115200 bauds).

`soukoukou` (le_loto) utilise déjà `/dev/ttyTHS1` : le câblage doit déjà être en place. **À vérifier.**

### b. Arrêter l'ancien lanceur soukoukou

Sinon deux programmes ouvrent la manette et le VESC en même temps.

```bash
crontab -l ; sudo crontab -l          # chercher la ligne @reboot de le_loto/RoboCar
crontab -e                            # (ou sudo crontab -e) : commenter cette ligne
pkill -f le_loto/RoboCar/binary       # l'arrêter tout de suite
pgrep -af le_loto                     # doit ne rien afficher
```

Pour revenir à soukoukou plus tard : décommenter la ligne, et arrêter `control` (`docker compose ... down`).

### c. Console série

`ttyTHS1` ne doit pas servir de console :

```bash
systemctl is-active nvgetty           # inactive attendu (sinon : sudo systemctl disable --now nvgetty)
grep -o "console=[^ ]*" /proc/cmdline # ttyS0 attendu, pas ttyTHS1
ls -l /dev/ttyTHS1                    # crw-rw---- root dialout
```

### d. Règle udev

Copier le dépôt (§2), puis :

```bash
cd ~/Robocar_ROS2_wesc
sudo cp host/99-robocar.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/vesc                       # /dev/vesc -> ttyTHS1
```

Cette règle remplace celle de la version F1TENTH (plus de `/dev/vesc` sur l'USB). Pour revenir en arrière : recopier `host/99-robocar.rules` depuis `~/Robocar_ROS2`.

## 2. Copier le dépôt sur la Jetson

Depuis le PC, dans un **nouveau dossier**, à côté de l'ancien `~/Robocar_ROS2` :

```bash
rsync -a --delete --exclude .env --exclude ros2_ws/build --exclude ros2_ws/install --exclude ros2_ws/log \
  ~/Documents/Robocar_ROS2_wesc/ soukoukou@192.168.67.1:~/Robocar_ROS2_wesc/
```

Sur la Jetson, reprendre le `.env` existant (même `ROS_DOMAIN_ID`) :

```bash
cp ~/Robocar_ROS2/.env ~/Robocar_ROS2_wesc/.env    # ou : cp .env.example .env, puis remplir ROS_DOMAIN_ID
```

## 3. Construire l'image

Les deux dossiers produisent la même image, `robocar:jazzy`. Garder l'ancienne sous un autre nom avant le premier build :

```bash
docker tag robocar:jazzy robocar:jazzy-f1tenth     # une fois ; retour : docker tag robocar:jazzy-f1tenth robocar:jazzy
```

Puis, sur la Jetson :

```bash
cd ~/Robocar_ROS2_wesc
nohup bash -c "time docker compose build control" > ~/build_wesc.log 2>&1 &
tail -f ~/build_wesc.log              # nohup : le build survit à une coupure SSH
```

`docker compose` trouve `docker/compose.yaml` grâce à `COMPOSE_FILE` dans `.env`. Durée : environ 17 min pour la version F1TENTH (`apt` en cache ensuite). Fin attendue : `Summary: 11 packages finished`. Le seul avertissement attendu est `serial_port_baudrate` dans `ldlidar_stl_ros2`.

Après une modification du code : recopier (§2), puis `docker compose build control` et `docker compose up -d --force-recreate control`.

## 4. Lancer

Brancher la batterie, le dongle F710 (interrupteur au dos sur **X**, LED MODE éteinte), puis :

```bash
cd ~/Robocar_ROS2_wesc
docker compose --profile control up -d            # ajouter --profile lidar si le LiDAR est branché
docker compose logs -f control                    # Ctrl+C pour quitter les logs
docker compose down                               # tout arrêter
```

Au démarrage, dans les logs :

```
[robocar_vesc]: port /dev/vesc à 115200 bauds, mode speed, watchdog 0.10 s
[robocar_vesc]: /dev/vesc ouvert
[robocar_teleop]: manette "<nom du pilote>" sur /dev/input/by-id/usb-Logitech_Wireless_Gamepad_F710_...-event-joystick
```

Commandes ROS sur la Jetson :

```bash
rc() { docker compose -f ~/Robocar_ROS2_wesc/docker/compose.yaml exec control /entrypoint.sh "$@"; }
rc ros2 node list                         # robocar_teleop, ackermann_mux, robocar_vesc, vesc_to_odom_node, robot_state_publisher
rc ros2 topic echo /sensors/core --once   # voltage_input ≈ LiPo, fault_code 0
rc ros2 topic echo /teleop/state          # boutons et rampe en direct
```

Depuis le PC (Jazzy natif, `README.md` §2), les mêmes commandes `ros2` marchent sans `rc`.

## 5. Manette

| Commande | Effet |
|---|---|
| Rien | Arrêt, même si un nœud publie `/drive` |
| **LB maintenu** + **RT** | Conduite manuelle : accélère, avec une rampe douce (expo 2,0) |
| LB + **LT** | Freine, d'autant plus fort que la gâchette est enfoncée |
| LB + stick gauche horizontal | Direction |
| **Y** (appui) | Bascule avant / arrière ; la vitesse repasse par 0 avec une pause de 0,2 s |
| **Home** | Consigne nulle immédiate |
| **RB maintenu** | Un nœud étudiant conduit : `/drive` passe |
| Manette perdue | Plus rien n'est publié : arrêt en moins de 0,2 s |

Relâcher LB remet la rampe à 0 : un nouvel appui repart de l'arrêt. Vitesse max : `max_speed` = 1 m/s (`teleop.yaml` et `vesc.yaml`). Le dongle peut être débranché et rebranché sans redémarrer le conteneur.

## 6. Premier essai, ROUES EN L'AIR

Dans l'ordre ; ne passer à la ligne suivante que si la précédente est bonne.

| # | Essai | Attendu |
|---|---|---|
| 1 | Lancer `control`, ne rien toucher | `/sensors/core` publie à ~50 Hz (`rc ros2 topic hz /sensors/core`), `voltage_input` ≈ tension LiPo. Rien ne bouge |
| 2 | `rc ros2 topic echo /teleop/state`, appuyer sur LB, RB, RT, LT, Y | `manual_enabled`, `drive_enabled`, `throttle`, `direction` suivent les boutons |
| 3 | LB + un peu de RT | Les roues tournent **vers l'avant**. Sinon : en mode `speed`, inverser le signe de `speed_to_erpm_gain` dans `vesc.yaml` (pas `invert_motor`, qui ne sert qu'en mode `duty`) |
| 4 | LB + stick gauche à gauche | Les roues braquent **à gauche**. Sinon : inverser le signe de `steering_angle_to_servo_gain` |
| 5 | Relâcher LB en roulant | Arrêt |
| 6 | `rc ros2 topic pub -r 20 /drive ackermann_msgs/msg/AckermannDriveStamped "{drive: {speed: 0.5}}"` | Rien sans RB ; les roues tournent avec RB ; arrêt au relâchement |
| 7 | Même chose, RB maintenu, Ctrl+C sur le `pub` | Arrêt en moins de 0,2 s (watchdog de `robocar_vesc`) |
| 8 | RB maintenu, `docker compose kill control` | Arrêt en 0,5 s au plus (timeout du VESC : plus d'`ALIVE`) |
| 9 | Débrancher le dongle en roulant | Arrêt ; au rebranchement, `manette "..."` revient dans les logs |

Si le moteur est saccadé à basse vitesse en mode `speed` (moteur sans capteur), passer `motor_mode: duty` dans `vesc.yaml` : c'est le comportement de soukoukou (zone morte 6 %, `duty_max` 20 %). Recommencer alors les essais 3 à 7.

Les résultats sont à reporter dans le tableau « Chaîne de sécurité » de `ARCHITECTURE.md`. `tests/test_safety_chain.py` écoute encore les topics de la version F1TENTH (`/joy`, `commands/motor/*`) : il est à adapter avant de pouvoir servir ici.

## 7. Tester sans la voiture (PC)

```bash
cd ~/Documents/Robocar_ROS2_wesc
docker build -f docker/Dockerfile -t robocar:jazzy .       # image amd64, ne sert pas sur la Jetson
# chaîne complète sans manette ni VESC : les nœuds tournent et réessaient d'ouvrir les périphériques
docker run --rm -it --network host -e ROS_DOMAIN_ID=42 robocar:jazzy \
  ros2 launch robocar_bringup control.launch.py
# tests de wesc (sans matériel)
docker run --rm -v $PWD/ros2_ws/src/wesc:/ws/src/wesc:ro ros:jazzy-ros-base bash -c \
  'apt-get update -qq && apt-get install -y -qq ros-jazzy-ament-cmake-gtest > /dev/null;
   source /opt/ros/jazzy/setup.bash && cd /ws && colcon build && colcon test && colcon test-result --verbose'
```

Arguments de `control.launch.py` : `with_teleop:=false` (sans manette, publier soi-même `/teleop` et `/drive_enable`), `with_vesc:=false` (sans VESC).

## 8. Dépannage

| Symptôme | Cause probable | Solution |
|---|---|---|
| `control` refuse de démarrer : `/dev/vesc` introuvable | Règle udev pas installée | §1 d ; `ls -l /dev/vesc` doit montrer `ttyTHS1` |
| `ouverture de /dev/vesc impossible : Device or resource busy` | soukoukou ou VESC Tool tient le port | §1 b ; `sudo fuser -v /dev/ttyTHS1` |
| `réponse GET_VALUES : Connection timed out` en boucle | Câblage TX/RX inversé, VESC éteint, mauvais débit, ou console série sur `ttyTHS1` | §1 a et c ; vérifier `app_uart_baudrate` = 115200 dans VESC Tool |
| `aucune F710 dans /dev/input/by-id` | Dongle absent, ou interrupteur sur **D** | Interrupteur sur **X** ; `ls /dev/input/by-id/` sur la Jetson |
| La manette ne fait rien | LB pas maintenu, ou deux programmes lisent la manette | `rc ros2 topic echo /teleop/state` ; §1 b |
| Les roues tournent à l'envers / braquent à l'envers | Signe des gains | §6, essais 3 et 4 |
| Le moteur ne s'arrête qu'après 0,5 s quand on coupe `/drive` | Watchdog inactif | Logs de `robocar_vesc` : `rien sur ackermann_cmd depuis 0.10 s` doit apparaître |
| Revenir à la version F1TENTH | — | `down` ici, `docker tag robocar:jazzy-f1tenth robocar:jazzy`, ancienne règle udev (§1 d), USB du VESC rebranché, puis lancer depuis `~/Robocar_ROS2` |
