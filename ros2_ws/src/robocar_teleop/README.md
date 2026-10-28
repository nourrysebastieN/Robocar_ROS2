# robocar_teleop

Nœud ROS 2 (C++) qui lit la F710 directement par **libevdev** et applique la logique de conduite de `soukoukou` (`le_loto/RoboCar/src/control/speed.c`). Remplace `joy` et `joy_teleop`.

> **Écrit, compilé, pas encore essayé sur la voiture.** Conception : `docs/WESC_JAZZY.md` §3 et §5.

## Interface

| Sortie | Type | Fréquence | Quand |
|---|---|---|---|
| `teleop` | `ackermann_msgs/AckermannDriveStamped` | 50 Hz | Toujours si la manette est là : consigne de la rampe si LB est maintenu, sinon 0 |
| `drive_enable` | `std_msgs/Bool` (`false` = déverrouillé) | 50 Hz | Seulement tant que RB est maintenu (même convention que la config `joy_teleop` actuelle) |
| `teleop/state` | `robocar_msgs/TeleopState` | 20 Hz | Toujours (collecte de données) |

Manette absente ou débranchée : **plus rien n'est publié**. `ackermann_mux` masque alors `/drive` (lock expiré) puis se tait, et le watchdog de `robocar_vesc` met la vitesse à 0.

## Correspondance avec `soukoukou`

Unités : `soukoukou` travaille en duty VESC (±100000 = 100 %), entre la zone morte (6000) et `MAX_SPEED` (20000), soit une plage utile de 14000. Ici, la rampe travaille en **fraction de `max_speed`** ; la zone morte est gérée par `robocar_vesc` (mode `duty`).

| `soukoukou` | Valeur | Paramètre ici | Défaut (converti) |
|---|---|---|---|
| `THROTTLE_CURVE` | 2,0 | `throttle_curve` | `2.0` |
| `ACCEL_STEP` (par 50 ms) | 400 | `accel_step` | `0.029` (400 / 14000) |
| `DECEL_STEP` (par 50 ms) | 1000 | `decel_step` | `0.071` (1000 / 14000) |
| Frein : `DECEL_STEP × 4 × (1 + 2 × L2)` | 4000 à 12000 | `brake_step` | `0.286` (× 1 à 3 selon L2) |
| Passage par zéro : `DECEL_STEP × 5` | 5000 | `cross_step` | `0.357` |
| `CROSS_HOLD_TICKS` | 4 (0,2 s) | `cross_hold_ticks` | `4` |
| `SPEED_STEP` (période de la rampe) | 50 ms | `ramp_rate` | `20.0` Hz |
| Direction : stick X / 32768 | −1..1 | `max_steering_angle` | `0.34` rad (comme `joy_teleop.yaml`) |
| — | — | `max_speed` | `1.0` m/s (même valeur que `robocar_vesc`) |

Consigne publiée : `speed = signe(sens) × fraction × max_speed`, `steering_angle = −X × max_steering_angle` (REP-103 : angle positif = gauche ; à vérifier, le stick à gauche donne X négatif sous evdev).

## Boutons (evdev, F710 en mode X)

| Code evdev | Bouton | Rôle |
|---|---|---|
| `BTN_TL` | LB | Homme mort conduite manuelle |
| `BTN_TR` | RB | Déverrouille `/drive` (lock `drive_enable`) |
| `ABS_RZ` (0..255) | RT / R2 | Accélérer |
| `ABS_Z` (0..255) | LT / L2 | Freiner |
| `ABS_X` (±32768) | stick gauche horizontal | Direction |
| `BTN_WEST` (= `BTN_Y`, `TRIANGLE` dans `le_loto/RoboCar/include/input.h`) | Y | Bascule avant / arrière, sur l'appui seulement (comme R1 dans `soukoukou`) |
| `BTN_MODE` | Home | Consigne nulle immédiate, cible et vitesse remises à 0 |

**R1 ne sert plus à inverser le sens** (c'était le cas dans `soukoukou`) : RB est le bouton d'homme mort de `/drive`, et le partager rendrait la sécurité ambiguë.

Les numéros sont à relever avec `evtest` dans le conteneur, comme pour `joy_teleop.yaml` (2026-09-25).

## Paramètres

| Paramètre | Défaut |
|---|---|
| `device` | `""` : premier `/dev/input/by-id/*F710*-event-joystick` trouvé (sinon un chemin précis, ex. `…F710_B76691B2-event-joystick` sur `soukoukou`) |
| `reconnect_period` | `2.0` s (comme `get_controller_dev`) |
| `publish_rate` | `50.0` Hz |
| `state_rate` | `20.0` Hz |
| `stick_deadzone` | `0.05` (comme `deadzone` de `joy`) |
| Rampe et direction | tableau ci-dessus |

Sans LB, la rampe est remise à 0 : un nouvel appui sur LB repart de l'arrêt. Les bornes des axes sont lues dans le pilote (`libevdev_get_abs_info`), pas supposées.

## Différences voulues avec `soukoukou`

- Ne quitte pas sur Home : le nœud tourne tant que le service `control` tourne.
- N'écrit jamais sur le VESC : seulement des topics. `robocar_vesc` est le seul à ouvrir le port.
- Pas d'`action_bridge` TCP (port 5008) : `teleop/state` le remplace. `real_car_client.py` (LIAM) est à porter en `rclpy`, ou un petit pont `teleop/state` → TCP 5008 peut être ajouté si besoin de compatibilité.
- Pas de lanceur de scripts (`binary/init`, boutons Triangle / Carré / Croix).
