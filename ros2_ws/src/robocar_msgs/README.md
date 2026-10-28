# robocar_msgs

Messages propres à la Robocar.

> **Écrit, compilé, pas encore essayé sur la voiture.** Conception : `docs/WESC_JAZZY.md` §3.

## `msg/TeleopState.msg`

Publié par `robocar_teleop` sur `/teleop/state`. Remplace l'`action_bridge` TCP de `soukoukou` (port 5008), utilisé par `real_car_client.py` (dépôt LIAM) pour la collecte de données.

```
std_msgs/Header header

# Manette brute, même formule que l'action_bridge de soukoukou
float32 throttle        # RT - LT, de -1 à 1
float32 steer           # stick gauche X, de -1 à 1, + = droite

# État de la logique de conduite
uint8 DIRECTION_FORWARD = 0
uint8 DIRECTION_REVERSE = 1
uint8 direction
bool braking
float32 target_fraction # cible de la rampe, de 0 à 1
float32 fraction        # valeur courante de la rampe, de 0 à 1

# Boutons de sécurité
bool manual_enabled     # LB maintenu
bool drive_enabled      # RB maintenu
```

Construction : `ament_cmake` + `rosidl_default_generators`, dépendance `std_msgs`.
