#include "bsp_sim.h"

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

static struct timespec g_t0;

static int g_tcp = -1;
static bool g_tcp_connected;
static uint32_t g_t_try_us;
static bool g_tried;
static struct sockaddr_in g_tcp_addr;

static int g_udp = -1;

static uint8_t g_flash[2][BSP_FLASH_SLOT_SIZE];
static char g_flash_path[512];

void bsp_sim_init(void)
{
    clock_gettime(CLOCK_MONOTONIC, &g_t0);
}

uint32_t bsp_time_us(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    const int64_t us = (int64_t)(t.tv_sec - g_t0.tv_sec) * 1000000 + (t.tv_nsec - g_t0.tv_nsec) / 1000;
    return (uint32_t)us;
}

void bsp_wait(uint32_t timeout_us)
{
    struct pollfd p[2];
    nfds_t n = 0;
    if (g_tcp >= 0 && g_tcp_connected) {
        p[n].fd = g_tcp;
        p[n].events = POLLIN;
        n++;
    }
    if (g_udp >= 0) {
        p[n].fd = g_udp;
        p[n].events = POLLIN;
        n++;
    }
    if (n == 0u) {
        const struct timespec ts = {0, (long)timeout_us * 1000L};
        nanosleep(&ts, NULL);
        return;
    }
    /* poll() has millisecond resolution; sub-millisecond waits become a short sleep. */
    const int ms = (int)(timeout_us / 1000u);
    if (ms == 0) {
        const struct timespec ts = {0, (long)timeout_us * 1000L};
        nanosleep(&ts, NULL);
        return;
    }
    (void)poll(p, n, ms);
}

/* ---- TCP ---- */

void bsp_uart_config(const char *host, uint16_t port)
{
    memset(&g_tcp_addr, 0, sizeof g_tcp_addr);
    g_tcp_addr.sin_family = AF_INET;
    g_tcp_addr.sin_port = htons(port);
    if (inet_pton(AF_INET, host, &g_tcp_addr.sin_addr) != 1) {
        fprintf(stderr, "tof_sil: bad host %s\n", host);
    }
}

static void tcp_close(void)
{
    if (g_tcp >= 0) {
        close(g_tcp);
    }
    g_tcp = -1;
    g_tcp_connected = false;
}

bool bsp_uart_poll(uint32_t now_us)
{
    if (g_tcp_connected) {
        return true;
    }
    if (g_tcp >= 0) {
        /* Non-blocking connect in progress. */
        struct pollfd p = {g_tcp, POLLOUT, 0};
        if (poll(&p, 1, 0) > 0) {
            int err = 0;
            socklen_t len = sizeof err;
            getsockopt(g_tcp, SOL_SOCKET, SO_ERROR, &err, &len);
            if (err == 0) {
                g_tcp_connected = true;
                fprintf(stderr, "tof_sil: FC UART connected\n");
                return true;
            }
            tcp_close();
        } else if ((uint32_t)(now_us - g_t_try_us) > 500000u) {
            tcp_close();
        } else {
            return false;
        }
    }
    if (g_tried && (uint32_t)(now_us - g_t_try_us) < 500000u) {
        return false;
    }
    g_tried = true;
    g_t_try_us = now_us;
    g_tcp = socket(AF_INET, SOCK_STREAM, 0);
    if (g_tcp < 0) {
        return false;
    }
    const int one = 1;
    setsockopt(g_tcp, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
    fcntl(g_tcp, F_SETFL, fcntl(g_tcp, F_GETFL, 0) | O_NONBLOCK);
    if (connect(g_tcp, (const struct sockaddr *)&g_tcp_addr, sizeof g_tcp_addr) == 0) {
        g_tcp_connected = true;
        fprintf(stderr, "tof_sil: FC UART connected\n");
        return true;
    }
    if (errno != EINPROGRESS) {
        tcp_close();
    }
    return false;
}

size_t bsp_uart_read(uint8_t *dst, size_t max)
{
    if (!g_tcp_connected) {
        return 0u;
    }
    const ssize_t n = recv(g_tcp, dst, max, 0);
    if (n > 0) {
        return (size_t)n;
    }
    if (n == 0 || (errno != EAGAIN && errno != EWOULDBLOCK)) {
        fprintf(stderr, "tof_sil: FC UART disconnected\n");
        tcp_close();
    }
    return 0u;
}

void bsp_uart_write(const uint8_t *src, size_t len)
{
    size_t off = 0;
    while (g_tcp_connected && off < len) {
        const ssize_t n = send(g_tcp, src + off, len - off, MSG_NOSIGNAL);
        if (n > 0) {
            off += (size_t)n;
        } else if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            struct pollfd p = {g_tcp, POLLOUT, 0};
            (void)poll(&p, 1, 10);
        } else {
            tcp_close();
        }
    }
}

/* ---- UDP ---- */

bool bsp_udp_open(uint16_t port)
{
    g_udp = socket(AF_INET, SOCK_DGRAM, 0);
    if (g_udp < 0) {
        return false;
    }
    const int rcvbuf = 1 << 20;
    setsockopt(g_udp, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof rcvbuf);
    struct sockaddr_in a;
    memset(&a, 0, sizeof a);
    a.sin_family = AF_INET;
    a.sin_port = htons(port);
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (bind(g_udp, (const struct sockaddr *)&a, sizeof a) != 0) {
        close(g_udp);
        g_udp = -1;
        return false;
    }
    fcntl(g_udp, F_SETFL, fcntl(g_udp, F_GETFL, 0) | O_NONBLOCK);
    return true;
}

size_t bsp_udp_read(uint8_t *dst, size_t max)
{
    if (g_udp < 0) {
        return 0u;
    }
    const ssize_t n = recv(g_udp, dst, max, 0);
    return (n > 0) ? (size_t)n : 0u;
}

/* ---- flash ---- */

static void flash_store(void)
{
    if (g_flash_path[0] == '\0') {
        return;
    }
    FILE *f = fopen(g_flash_path, "wb");
    if (f != NULL) {
        (void)fwrite(g_flash, 1, sizeof g_flash, f);
        fclose(f);
    }
}

static bool flash_erase(void *ctx, uint8_t slot)
{
    (void)ctx;
    if (slot > 1u) {
        return false;
    }
    memset(g_flash[slot], 0xFF, BSP_FLASH_SLOT_SIZE);
    flash_store();
    return true;
}

static bool flash_write(void *ctx, uint8_t slot, uint32_t offset, const uint8_t *src, uint32_t len)
{
    (void)ctx;
    if (slot > 1u || offset + len > BSP_FLASH_SLOT_SIZE) {
        return false;
    }
    for (uint32_t k = 0; k < len; k++) {
        g_flash[slot][offset + k] &= src[k]; /* NOR flash: bits only go from 1 to 0 */
    }
    flash_store();
    return true;
}

static bool flash_read(void *ctx, uint8_t slot, uint32_t offset, uint8_t *dst, uint32_t len)
{
    (void)ctx;
    if (slot > 1u || offset + len > BSP_FLASH_SLOT_SIZE) {
        return false;
    }
    memcpy(dst, &g_flash[slot][offset], len);
    return true;
}

void bsp_flash_init(params_flash_t *f, const char *path)
{
    memset(g_flash, 0xFF, sizeof g_flash);
    g_flash_path[0] = '\0';
    if (path != NULL) {
        snprintf(g_flash_path, sizeof g_flash_path, "%s", path);
        FILE *fp = fopen(path, "rb");
        if (fp != NULL) {
            (void)fread(g_flash, 1, sizeof g_flash, fp);
            fclose(fp);
        }
    }
    f->ctx = NULL;
    f->slot_size = BSP_FLASH_SLOT_SIZE;
    f->erase = flash_erase;
    f->write = flash_write;
    f->read = flash_read;
}
