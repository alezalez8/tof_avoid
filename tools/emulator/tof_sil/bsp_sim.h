#ifndef TOF_SIL_BSP_SIM_H
#define TOF_SIL_BSP_SIM_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "core/params.h"

/* Simulated board for the software-in-the-loop module (E3): POSIX time, the FC UART as a TCP client
 * to a SITL SERIAL port, sensor frames over UDP, parameter flash in a file. Host-only; the core is
 * linked unchanged. */

/* Microseconds since bsp_sim_init(), wrapping like the MCU timer. */
uint32_t bsp_time_us(void);

void bsp_sim_init(void);

/* Sleeps until a socket is readable or timeout_us elapses. */
void bsp_wait(uint32_t timeout_us);

/* ---- FC UART: TCP client ---- */

/* Target of the connection; connecting is retried by bsp_uart_poll(). */
void bsp_uart_config(const char *host, uint16_t port);

/* Keeps the connection up (reconnect every 500 ms). True while connected. */
bool bsp_uart_poll(uint32_t now_us);

/* Received bytes (non-blocking); 0 if none or not connected. */
size_t bsp_uart_read(uint8_t *dst, size_t max);

/* Writes bytes (the end of a DMA transfer); dropped if not connected. */
void bsp_uart_write(const uint8_t *src, size_t len);

/* ---- sensor frames: UDP server ---- */

bool bsp_udp_open(uint16_t port);

/* One datagram (non-blocking); 0 if none. */
size_t bsp_udp_read(uint8_t *dst, size_t max);

/* ---- parameter flash: two slots in a file (missing file = erased flash) ---- */

#define BSP_FLASH_SLOT_SIZE 1024u

void bsp_flash_init(params_flash_t *f, const char *path);

#endif /* TOF_SIL_BSP_SIM_H */
