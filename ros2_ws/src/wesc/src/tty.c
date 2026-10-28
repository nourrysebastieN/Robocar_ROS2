#include "wesc/wesc.h"
#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

static int baud_to_speed(int baud, speed_t *speed)
{
    switch (baud) {
    case 9600: *speed = B9600; return 0;
    case 19200: *speed = B19200; return 0;
    case 38400: *speed = B38400; return 0;
    case 57600: *speed = B57600; return 0;
    case 115200: *speed = B115200; return 0;
    case 230400: *speed = B230400; return 0;
    case 460800: *speed = B460800; return 0;
    case 921600: *speed = B921600; return 0;
    default: return -1;
    }
}

/**
 * @brief Ouvre le port série et le passe en 8N1 brut, sans contrôle de flux.
 * @param path Chemin du port (ex. /dev/ttyTHS1).
 * @param baud Débit en bauds.
 * @return Le descripteur de fichier, ou -1 et errno.
 */
int wesc_open(const char *path, int baud)
{
    struct termios tty;
    speed_t speed;
    int fd;
    int saved;

    if (path == NULL || baud_to_speed(baud, &speed) == -1) {
        errno = EINVAL;
        return -1;
    }
    fd = open(path, O_RDWR | O_NOCTTY | O_CLOEXEC);
    if (fd == -1)
        return -1;
    if (tcgetattr(fd, &tty) == -1)
        goto fail;
    cfmakeraw(&tty);
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
    tty.c_cflag |= CS8;
    // read() rend la main tout de suite : les délais sont gérés par poll()
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;
    if (tcsetattr(fd, TCSANOW, &tty) == -1)
        goto fail;
    if (tcflush(fd, TCIOFLUSH) == -1)
        goto fail;
    return fd;
fail:
    saved = errno;
    close(fd);
    errno = saved;
    return -1;
}

void wesc_close(int fd)
{
    if (fd >= 0)
        close(fd);
}
