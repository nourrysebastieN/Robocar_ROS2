# wesc

Protocole VESC en C (remplace pyvesc et `vesc_driver`). Bibliothèque statique, **sans dépendance ROS** : utilisée par `robocar_vesc`, et compilable hors ROS (`le_loto/RoboCar`).

> **À écrire.** Ce dossier n'a pas encore de `package.xml` : colcon l'ignore. Conception : `docs/WESC_JAZZY.md` §3.

## Origine

| | |
|---|---|
| Source | `~/le_loto/wesc` sur la Jetson (`src/crc16.c`, `src/write.c`, `src/read.c`, `include/wesc.h`) |
| Couche série | `~/le_loto/RoboCar/src/control/tty.c` (`set_term`, `send_alive`, `init`), à intégrer ici |
| Dépôt git | aucun pour `wesc` seul ; `le_loto/RoboCar` : `git@github.com:TheSoukoukou/RoboCar.git` |

À la copie, noter la date et le commit de `RoboCar` utilisé, comme dans `third_party/VENDORED.md`.

## Commandes VESC couvertes

| Commande | ID VESC | Charge utile | État |
|---|---|---|---|
| `COMM_ALIVE` | 30 (`0x1e`) | — | existe |
| `COMM_GET_VALUES` | 4 | réponse décodée dans `get_values_t` | existe |
| `COMM_SET_DUTY` | 5 | int32 BE, duty × 100000 | existe |
| `COMM_SET_CURRENT` | 6 | int32 BE, A × 1000 | existe |
| `COMM_SET_CURRENT_BRAKE` | 7 | int32 BE, A × 1000 | existe |
| `COMM_SET_RPM` | 8 | int32 BE, ERPM | existe |
| `COMM_SET_SERVO_POS` | 12 (`0x0c`) | int16 BE, position × 1000 | existe |
| `COMM_FW_VERSION` | 0 | — | à ajouter (vérifier la liaison au démarrage, comme `vesc_driver`) |

Trame courte : `0x02`, longueur (1 octet), charge utile, CRC16-CCITT (poly `0x1021`, sur la charge utile seulement), `0x03`.

## Changements prévus

1. Couche série dans la bibliothèque : `int wesc_open(const char *path, speed_t baud)`, `void wesc_close(int fd)`.
2. Préfixe `wesc_` sur tous les symboles publics. Les noms actuels (`init`, `alive`, `crc16`, `cmd_list`) restent comme alias derrière `WESC_LEGACY_NAMES`, pour `le_loto`.
3. `extern "C"` dans `wesc.h`.
4. Plus de `fork()` dans la bibliothèque : `wesc_send_alive(fd)` envoie **un** `ALIVE`, c'est l'appelant qui choisit quand.
5. `wesc_read_packet(fd, buf, size, timeout_ms)` : `poll()` avec délai, resynchronisation sur `0x02`, trames longues (`0x03`, longueur sur 2 octets) ignorées proprement.
6. Fonctions typées en unités physiques :
   ```c
   int wesc_set_duty(int fd, float duty);        // -1..1
   int wesc_set_rpm(int fd, int32_t erpm);
   int wesc_set_current(int fd, float amps);
   int wesc_set_brake(int fd, float amps);
   int wesc_set_servo(int fd, float pos);        // 0..1
   int wesc_request_values(int fd);
   int wesc_read_values(int fd, wesc_values_t *out, int timeout_ms);
   ```
7. Valeurs de retour homogènes : `0` ou nombre d'octets en cas de succès, `-1` et `errno` sinon (plus de `fprintf` dans la bibliothèque).
8. Contrôles de débordement de `write.c` unifiés (aujourd'hui `>= 256` ou `> 256` selon la commande).

## Construction

`ament_cmake`, cible `wesc::wesc` exportée (`ament_export_targets`). Dans un autre paquet :

```cmake
find_package(wesc REQUIRED)
target_link_libraries(robocar_vesc_node wesc::wesc)
```

Un `Makefile` reste fourni pour la compilation hors ROS (`make` → `libwesc.a`).

## Tests (sans matériel)

- CRC16 : vecteurs connus.
- Encodage : chaque commande, octet par octet, comparé à une trame de référence (VESC Tool ou pyvesc).
- Décodage : une réponse `GET_VALUES` enregistrée sur la voiture (firmware 7.00) ; vérifier `v_in` ≈ 15,7 V, et pourquoi `temp_fet` / `temp_motor` valent 0 (déjà vu avec `vesc_driver`).
- Lecture : trame tronquée, CRC faux, octets parasites avant `0x02`, timeout.
