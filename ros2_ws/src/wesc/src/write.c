#include "wesc/wesc.h"
#include <errno.h>
#include <math.h>
#include <unistd.h>

static void put_int16(uint8_t *buffer, int16_t value)
{
    buffer[0] = (uint8_t)((uint16_t)value >> 8);
    buffer[1] = (uint8_t)value;
}

static void put_int32(uint8_t *buffer, int32_t value)
{
    buffer[0] = (uint8_t)((uint32_t)value >> 24);
    buffer[1] = (uint8_t)((uint32_t)value >> 16);
    buffer[2] = (uint8_t)((uint32_t)value >> 8);
    buffer[3] = (uint8_t)value;
}

// Arrondi borné à l'intervalle d'un int32 (évite le comportement indéfini de la conversion)
static int32_t scale_int32(float value, float scale)
{
    double scaled = (double)value * scale;

    if (isnan(scaled))
        return 0;
    if (scaled > INT32_MAX)
        return INT32_MAX;
    if (scaled < INT32_MIN)
        return INT32_MIN;
    return (int32_t)lround(scaled);
}

static float clampf(float value, float min, float max)
{
    if (isnan(value))
        return 0.0f;
    return value < min ? min : (value > max ? max : value);
}

/**
 * @brief Encadre la charge utile (début, longueur, CRC, fin) et écrit la trame d'un seul write().
 * @return Le nombre d'octets écrits, ou -1 et errno.
 */
ssize_t wesc_write_payload(int fd, const uint8_t *payload, size_t len)
{
    uint8_t frame[WESC_MAX_PAYLOAD + 6];
    size_t index = 0;
    uint16_t crc;
    ssize_t written;

    if (payload == NULL || len == 0 || len > WESC_MAX_PAYLOAD) {
        errno = EINVAL;
        return -1;
    }
    if (len < 256) {
        frame[index++] = 0x02;
        frame[index++] = (uint8_t)len;
    } else {
        frame[index++] = 0x03;
        frame[index++] = (uint8_t)(len >> 8);
        frame[index++] = (uint8_t)len;
    }
    for (size_t i = 0; i < len; i++)
        frame[index++] = payload[i];
    crc = wesc_crc16(payload, len);
    frame[index++] = (uint8_t)(crc >> 8);
    frame[index++] = (uint8_t)crc;
    frame[index++] = 0x03;
    do
        written = write(fd, frame, index);
    while (written == -1 && errno == EINTR);
    if (written >= 0 && (size_t)written != index) {
        errno = EIO;
        return -1;
    }
    return written;
}

static int send_int32(int fd, wesc_cmd_t cmd, int32_t value)
{
    uint8_t payload[5] = {(uint8_t)cmd};

    put_int32(payload + 1, value);
    return wesc_write_payload(fd, payload, sizeof(payload)) == -1 ? -1 : 0;
}

static int send_cmd(int fd, wesc_cmd_t cmd)
{
    uint8_t payload[1] = {(uint8_t)cmd};

    return wesc_write_payload(fd, payload, sizeof(payload)) == -1 ? -1 : 0;
}

int wesc_send_alive(int fd)
{
    return send_cmd(fd, WESC_COMM_ALIVE);
}

int wesc_request_values(int fd)
{
    return send_cmd(fd, WESC_COMM_GET_VALUES);
}

int wesc_set_duty(int fd, float duty)
{
    return send_int32(fd, WESC_COMM_SET_DUTY, scale_int32(clampf(duty, -1.0f, 1.0f), 100000.0f));
}

int wesc_set_current(int fd, float amps)
{
    return send_int32(fd, WESC_COMM_SET_CURRENT, scale_int32(amps, 1000.0f));
}

int wesc_set_brake(int fd, float amps)
{
    return send_int32(fd, WESC_COMM_SET_CURRENT_BRAKE, scale_int32(amps, 1000.0f));
}

int wesc_set_rpm(int fd, int32_t erpm)
{
    return send_int32(fd, WESC_COMM_SET_RPM, erpm);
}

int wesc_set_servo(int fd, float pos)
{
    uint8_t payload[3] = {(uint8_t)WESC_COMM_SET_SERVO_POS};

    put_int16(payload + 1, (int16_t)scale_int32(clampf(pos, 0.0f, 1.0f), 1000.0f));
    return wesc_write_payload(fd, payload, sizeof(payload)) == -1 ? -1 : 0;
}
