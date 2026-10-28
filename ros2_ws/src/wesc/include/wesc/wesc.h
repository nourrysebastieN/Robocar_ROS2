#ifndef WESC_H
    #define WESC_H
#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Protocole VESC par UART, trames courtes et longues.
 *
 *   courte : 0x02, longueur (1 octet), charge utile, CRC16 (2 octets), 0x03
 *   longue : 0x03, longueur (2 octets), charge utile, CRC16 (2 octets), 0x03
 *
 * Toutes les fonctions renvoient 0 (ou un nombre d'octets) en cas de succès,
 * -1 et errno sinon. Aucune n'écrit sur stdout / stderr.
 */

// Identifiants de commande VESC (bldc, datatypes.h)
typedef enum wesc_cmd_e {
    WESC_COMM_FW_VERSION = 0,
    WESC_COMM_GET_VALUES = 4,
    WESC_COMM_SET_DUTY = 5,
    WESC_COMM_SET_CURRENT = 6,
    WESC_COMM_SET_CURRENT_BRAKE = 7,
    WESC_COMM_SET_RPM = 8,
    WESC_COMM_SET_SERVO_POS = 12,
    WESC_COMM_ALIVE = 30
} wesc_cmd_t;

#define WESC_MAX_PAYLOAD 512

// Réponse à COMM_GET_VALUES, en unités physiques
typedef struct wesc_values_s {
    float temp_fet;             // °C
    float temp_motor;           // °C
    float avg_motor_current;    // A
    float avg_input_current;    // A
    float avg_id;               // A
    float avg_iq;               // A
    float duty_cycle_now;       // -1..1
    int32_t rpm;                // ERPM
    float v_in;                 // V
    float amp_hours;            // Ah
    float amp_hours_charged;    // Ah
    float watt_hours;           // Wh
    float watt_hours_charged;   // Wh
    int32_t tachometer;
    int32_t tachometer_abs;
    uint8_t mc_fault_code;
    float pid_pos_now;          // degrés
    uint8_t app_controller_id;
    // Présents seulement si la réponse est assez longue (firmware récent), 0 sinon
    float temp_mos1;            // °C
    float temp_mos2;            // °C
    float temp_mos3;            // °C
    float avg_vd;               // V
    float avg_vq;               // V
} wesc_values_t;

uint16_t wesc_crc16(const uint8_t *data, size_t len);

// Liaison série : 8N1, mode brut, sans contrôle de flux. baud : 9600 à 921600
int wesc_open(const char *path, int baud);
void wesc_close(int fd);

// Une trame complète autour de payload, écrite d'un coup
ssize_t wesc_write_payload(int fd, const uint8_t *payload, size_t len);

// Commandes, en unités physiques (bornées aux limites du protocole)
int wesc_send_alive(int fd);
int wesc_set_duty(int fd, float duty);          // -1..1
int wesc_set_current(int fd, float amps);
int wesc_set_brake(int fd, float amps);
int wesc_set_rpm(int fd, int32_t erpm);
int wesc_set_servo(int fd, float pos);          // 0..1
int wesc_request_values(int fd);

/*
 * Lit une trame et copie sa charge utile dans payload. timeout_ms < 0 : attente infinie.
 * Les octets avant un début de trame sont ignorés (resynchronisation).
 * errno : ETIMEDOUT, EBADMSG (CRC), EPROTO (fin de trame), EMSGSIZE, ECONNRESET.
 */
ssize_t wesc_read_packet(int fd, uint8_t *payload, size_t size, int timeout_ms);

// Décode une charge utile COMM_GET_VALUES (premier octet = 4)
int wesc_parse_values(const uint8_t *payload, size_t len, wesc_values_t *out);

// Lit jusqu'à une réponse COMM_GET_VALUES (les autres trames sont ignorées)
int wesc_read_values(int fd, wesc_values_t *out, int timeout_ms);

#ifdef WESC_LEGACY_NAMES
/*
 * Noms de la version le_loto, pour compiler RoboCar (soukoukou) sans changement.
 * Les valeurs gardent les anciennes unités : duty et courant × 100000 / × 1000, servo × 1000.
 */
typedef wesc_values_t get_values_t;
    #define WRITE_ALIVE(fd) wesc_send_alive(fd)
    #define WRITE_SET_SERVO(fd, value) wesc_set_servo(fd, (value) / 1000.0f)
    #define WRITE_SET_DUTY_CYCLE(fd, value) wesc_set_duty(fd, (value) / 100000.0f)
    #define WRITE_GET_VALUES(fd) wesc_request_values(fd)
    #define WRITE_SET_CURRENT_BRAKE(fd, value) wesc_set_brake(fd, (value) / 1000.0f)
    #define WRITE_SET_CURRENT(fd, value) wesc_set_current(fd, (value) / 1000.0f)
    #define WRITE_SET_RPM(fd, value) wesc_set_rpm(fd, (value))
    #define read_get_values(fd, values) wesc_read_values(fd, values, -1)
#endif

#ifdef __cplusplus
}
#endif

#endif
