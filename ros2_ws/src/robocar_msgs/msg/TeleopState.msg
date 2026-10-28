# Publié par robocar_teleop sur /teleop/state (remplace l'action_bridge TCP 5008 de soukoukou)
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
