#include "wesc/wesc.h"
#include <errno.h>
#include <poll.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

// Longueur minimale (jusqu'à app_controller_id) et complète (jusqu'à avg_vq)
#define GET_VALUES_MIN_LEN 59
#define GET_VALUES_FULL_LEN 73

static int64_t now_ms(void)
{
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

/**
 * @brief Lit exactement len octets avant l'échéance (deadline < 0 : pas d'échéance).
 * @return 0, ou -1 et errno (ETIMEDOUT, ECONNRESET si le port est fermé).
 */
static int read_exact(int fd, uint8_t *buffer, size_t len, int64_t deadline)
{
    size_t total = 0;

    while (total < len) {
        struct pollfd pfd = {.fd = fd, .events = POLLIN};
        int wait = -1;
        int ready;
        ssize_t bytes;

        if (deadline >= 0) {
            int64_t left = deadline - now_ms();

            if (left <= 0) {
                errno = ETIMEDOUT;
                return -1;
            }
            wait = (int)left;
        }
        ready = poll(&pfd, 1, wait);
        if (ready < 0) {
            if (errno == EINTR)
                continue;
            return -1;
        }
        if (ready == 0) {
            errno = ETIMEDOUT;
            return -1;
        }
        bytes = read(fd, buffer + total, len - total);
        if (bytes == 0) {
            errno = ECONNRESET;
            return -1;
        }
        if (bytes < 0) {
            if (errno == EINTR || errno == EAGAIN)
                continue;
            return -1;
        }
        total += (size_t)bytes;
    }
    return 0;
}

static ssize_t read_packet_until(int fd, uint8_t *payload, size_t size, int64_t deadline)
{
    uint8_t byte;
    uint8_t len_bytes[2];
    uint8_t tail[3];
    size_t length;
    uint16_t received_crc;

    // Resynchronisation : tout ce qui précède un octet de début est ignoré
    do {
        if (read_exact(fd, &byte, 1, deadline) == -1)
            return -1;
    } while (byte != 0x02 && byte != 0x03);
    if (byte == 0x02) {
        if (read_exact(fd, len_bytes, 1, deadline) == -1)
            return -1;
        length = len_bytes[0];
    } else {
        if (read_exact(fd, len_bytes, 2, deadline) == -1)
            return -1;
        length = ((size_t)len_bytes[0] << 8) | len_bytes[1];
    }
    if (length == 0 || length > size || length > WESC_MAX_PAYLOAD) {
        errno = EMSGSIZE;
        return -1;
    }
    if (read_exact(fd, payload, length, deadline) == -1)
        return -1;
    if (read_exact(fd, tail, sizeof(tail), deadline) == -1)
        return -1;
    if (tail[2] != 0x03) {
        errno = EPROTO;
        return -1;
    }
    received_crc = ((uint16_t)tail[0] << 8) | tail[1];
    if (received_crc != wesc_crc16(payload, length)) {
        errno = EBADMSG;
        return -1;
    }
    return (ssize_t)length;
}

ssize_t wesc_read_packet(int fd, uint8_t *payload, size_t size, int timeout_ms)
{
    if (payload == NULL || size == 0) {
        errno = EINVAL;
        return -1;
    }
    return read_packet_until(fd, payload, size, timeout_ms < 0 ? -1 : now_ms() + timeout_ms);
}

static int16_t get_int16(const uint8_t *buffer, size_t *index)
{
    uint16_t value = ((uint16_t)buffer[*index] << 8) | buffer[*index + 1];

    *index += 2;
    return (int16_t)value;
}

static int32_t get_int32(const uint8_t *buffer, size_t *index)
{
    uint32_t value = ((uint32_t)buffer[*index] << 24) | ((uint32_t)buffer[*index + 1] << 16)
        | ((uint32_t)buffer[*index + 2] << 8) | buffer[*index + 3];

    *index += 4;
    return (int32_t)value;
}

/**
 * @brief Décode COMM_GET_VALUES, même disposition que bldc (commands.c) et vesc_driver.
 * @return 0, ou -1 et errno (EINVAL, EPROTO si ce n'est pas GET_VALUES, EMSGSIZE si trop court).
 */
int wesc_parse_values(const uint8_t *payload, size_t len, wesc_values_t *out)
{
    size_t ind = 0;

    if (payload == NULL || out == NULL) {
        errno = EINVAL;
        return -1;
    }
    if (len < 1 || payload[ind++] != WESC_COMM_GET_VALUES) {
        errno = EPROTO;
        return -1;
    }
    if (len < GET_VALUES_MIN_LEN) {
        errno = EMSGSIZE;
        return -1;
    }
    memset(out, 0, sizeof(*out));
    out->temp_fet = get_int16(payload, &ind) / 10.0f;
    out->temp_motor = get_int16(payload, &ind) / 10.0f;
    out->avg_motor_current = get_int32(payload, &ind) / 100.0f;
    out->avg_input_current = get_int32(payload, &ind) / 100.0f;
    out->avg_id = get_int32(payload, &ind) / 100.0f;
    out->avg_iq = get_int32(payload, &ind) / 100.0f;
    out->duty_cycle_now = get_int16(payload, &ind) / 1000.0f;
    out->rpm = get_int32(payload, &ind);
    out->v_in = get_int16(payload, &ind) / 10.0f;
    out->amp_hours = get_int32(payload, &ind) / 10000.0f;
    out->amp_hours_charged = get_int32(payload, &ind) / 10000.0f;
    out->watt_hours = get_int32(payload, &ind) / 10000.0f;
    out->watt_hours_charged = get_int32(payload, &ind) / 10000.0f;
    out->tachometer = get_int32(payload, &ind);
    out->tachometer_abs = get_int32(payload, &ind);
    out->mc_fault_code = payload[ind++];
    out->pid_pos_now = get_int32(payload, &ind) / 1000000.0f;
    out->app_controller_id = payload[ind++];
    if (len < GET_VALUES_FULL_LEN)
        return 0;
    out->temp_mos1 = get_int16(payload, &ind) / 10.0f;
    out->temp_mos2 = get_int16(payload, &ind) / 10.0f;
    out->temp_mos3 = get_int16(payload, &ind) / 10.0f;
    out->avg_vd = get_int32(payload, &ind) / 1000.0f;
    out->avg_vq = get_int32(payload, &ind) / 1000.0f;
    return 0;
}

int wesc_read_values(int fd, wesc_values_t *out, int timeout_ms)
{
    uint8_t payload[WESC_MAX_PAYLOAD];
    int64_t deadline = timeout_ms < 0 ? -1 : now_ms() + timeout_ms;
    ssize_t len;

    if (out == NULL) {
        errno = EINVAL;
        return -1;
    }
    do {
        len = read_packet_until(fd, payload, sizeof(payload), deadline);
        if (len == -1)
            return -1;
    } while (payload[0] != WESC_COMM_GET_VALUES);
    return wesc_parse_values(payload, (size_t)len, out);
}
