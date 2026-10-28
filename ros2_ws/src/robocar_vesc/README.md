# robocar_vesc

Nœud ROS 2 (C++) entre `ackermann_mux` et le VESC, par la bibliothèque `wesc` sur l'UART de la Jetson. Remplace `ackermann_to_vesc` et `vesc_driver` (F1TENTH).

> **Écrit, compilé, pas encore essayé sur la voiture.** Conception : `docs/WESC_JAZZY.md` §3 et §6.

## Interface

| | Topic | Type | Remarque |
|---|---|---|---|
| Entrée | `ackermann_cmd` | `ackermann_msgs/AckermannDriveStamped` | Publié par `ackermann_mux` |
| Sortie | `sensors/core` | `vesc_msgs/VescStateStamped` | Depuis `GET_VALUES`, à `telemetry_rate` |
| Sortie | `sensors/servo_position_command` | `std_msgs/Float64` | Dernière position servo envoyée (lue par `vesc_to_odom`) |

Mêmes noms et types que `vesc_driver` : `vesc_to_odom` et les outils existants marchent sans changement.

Correspondance `get_values_t` (wesc) → `VescState` :

| wesc | VescState |
|---|---|
| `temp_fet`, `temp_motor` | `temp_fet`, `temp_motor` |
| `avg_motor_current`, `avg_input_current` | `current_motor`, `current_input` |
| `avg_id`, `avg_iq` | `avg_id`, `avg_iq` |
| `duty_cycle_now` | `duty_cycle` |
| `rpm` | `speed` (ERPM) |
| `v_in` | `voltage_input` |
| `amp_hours`, `amp_hours_charged` | `charge_drawn`, `charge_regen` |
| `watt_hours`, `watt_hours_charged` | `energy_drawn`, `energy_regen` |
| `tachometer`, `tachometer_abs` | `displacement`, `distance_traveled` |
| `mc_fault_code` | `fault_code` |
| `pid_pos_now` | `pid_pos_now` |
| `app_controller_id` | `controller_id` |

## Conversion

```
motor_mode = speed : erpm  = speed_to_erpm_gain × speed + speed_to_erpm_offset   → wesc_set_rpm
                     borné à [speed_min, speed_max]
motor_mode = duty  : f     = |speed| / max_speed (borné à 1)
                     duty  = 0 si f = 0, sinon duty_deadband + f × (duty_max − duty_deadband)
                     signe = celui de speed, inversé si invert_motor              → wesc_set_duty
servo              = steering_angle_to_servo_gain × steering_angle + offset       → wesc_set_servo
                     borné à [servo_min, servo_max]
```

En mode `duty`, le saut de la zone morte reprend `THROTTLE_DEADBAND` / `REVERSE_DEADBAND` de `soukoukou` (6 %).

Vitesse nulle (ou watchdog) : `SET_DUTY 0` dans les deux modes (moteur libre), pas `SET_RPM 0` qui fait vibrer un moteur sans capteur à l'arrêt.

`invert_motor` ne s'applique qu'en mode `duty`. En mode `speed`, le sens se règle par le signe de `speed_to_erpm_gain` : `vesc_to_odom` divise l'ERPM mesuré par ce même gain, l'odométrie reste donc juste.

## Paramètres

| Paramètre | Défaut | Origine |
|---|---|---|
| `port` | `/dev/vesc` (→ `ttyTHS1` par udev) | `soukoukou` : `UART_PATH` |
| `baud` | `115200` | `vesc_appconf.xml` : `app_uart_baudrate` |
| `motor_mode` | `speed` | `speed` ou `duty` |
| `speed_to_erpm_gain`, `speed_to_erpm_offset` | `4614.0`, `0.0` | `vesc.yaml` |
| `speed_min`, `speed_max` | `-5000.0`, `5000.0` (ERPM) | `vesc.yaml` |
| `max_speed` | `1.0` (m/s) | mode `duty` : vitesse qui donne `duty_max` |
| `duty_max` | `0.20` | `soukoukou` : `MAX_SPEED` 20000 / 100000 |
| `duty_deadband` | `0.06` | `soukoukou` : `THROTTLE_DEADBAND` 6000 / 100000 |
| `invert_motor` | `false` | à mesurer roues en l'air (`docs/WESC_JAZZY.md` §7) |
| `steering_angle_to_servo_gain`, `_offset` | `-0.88`, `0.50` | `vesc.yaml` |
| `servo_min`, `servo_max` | `0.20`, `0.80` | `vesc.yaml` |
| `cmd_timeout` | `0.1` (s) | watchdog, `vesc.yaml` |
| `telemetry_rate` | `50.0` (Hz) | fréquence de `vesc_driver` |
| `read_timeout_ms` | `20` | délai d'une réponse `GET_VALUES` |

Les gains communs avec `vesc_to_odom` restent dans `robocar_bringup/config/vesc.yaml`, section `/**` ; le reste dans la section `robocar_vesc`. `port` garde le nom `/dev/vesc` (lien udev vers `ttyTHS1`).

## Boucle (un seul fil)

Timer à `telemetry_rate` :
1. Si une consigne a moins de `cmd_timeout` : l'envoyer (moteur, puis servo si changé), puis `ALIVE`.
2. Sinon : moteur à 0, **pas d'`ALIVE`** (le timeout du VESC, 500 ms, reste actif).
3. `wesc_request_values`, `wesc_read_values` avec `read_timeout_ms`, publier `sensors/core`.
4. Erreur série : moteur à 0 si possible, log, réouverture du port au tick suivant.

Le callback `ackermann_cmd` ne fait que mémoriser la consigne et son heure (horloge monotone). Il n'écrit rien sur le port.

Arrêt du nœud : duty 0, servo au neutre (`offset`), fermeture du port.
