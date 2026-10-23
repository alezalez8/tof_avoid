/* tof_sil: the module core (core/, unchanged) on the host, for SITL (E3, TZ section 13).
 *
 * The FC UART is a TCP connection to a SITL SERIAL port, paced at UART_BAUD like the UART DMA; sensor
 * frames come over UDP from tools/emulator/scene.py (sil_proto.h) and are stamped on reception as the
 * INT would stamp them. The emulated driver performs the SNS-11 reinit handshake. Everything the
 * module does is written to a JSON Lines log (one object per line, field "k" is the record kind). */

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bsp_sim.h"
#include "core/crc32.h"
#include "core/params.h"
#include "core/phase.h"
#include "core/scheduler.h"
#include "core/version.h"
#include "sil_proto.h"

#define DMA_CHUNK 512u
#define MAX_SETS 64
#define STATUS_PERIOD_US 1000000u
#define LOOP_WAIT_US 1000u

typedef struct {
    const char *fc_host;
    uint16_t fc_port;
    uint16_t udp_port;
    const char *log_path;
    const char *flash_path;
    bool provision;
    uint32_t reinit_us;
    const char *sets[MAX_SETS];
    int n_sets;
} options_t;

typedef struct {
    bool powered;
    bool reinit_busy;
    uint32_t t_reinit_end_us;
    uint32_t reinits;
    uint32_t frames_dropped; /* received while re-initialising */
} sim_sensor_t;

static volatile sig_atomic_t g_quit;
static sched_t g_s;
static params_t g_p;
static FILE *g_log;

static void on_signal(int sig)
{
    (void)sig;
    g_quit = 1;
}

static void usage(void)
{
    fprintf(stderr,
            "usage: tof_sil [--fc HOST:PORT] [--udp PORT] [--log FILE] [--flash FILE]\n"
            "               [--set NAME=VALUE]... [--provision] [--reinit-ms N]\n"
            "  --fc        SITL SERIAL port of the module (default 127.0.0.1:5813)\n"
            "  --udp       UDP port for sensor frames from scene.py (default 14660)\n"
            "  --log       JSON Lines log (default stdout)\n"
            "  --flash     parameter flash file (missing = erased: both slots invalid, SW-07)\n"
            "  --set       parameter value in RAM (the debug console of the real module)\n"
            "  --provision write the parameters (with --set) to the flash file and exit\n"
            "  --reinit-ms duration of an emulated sensor reinit (default 300)\n");
}

static bool parse_args(int argc, char **argv, options_t *o)
{
    memset(o, 0, sizeof *o);
    o->fc_host = "127.0.0.1";
    o->fc_port = 5813u;
    o->udp_port = 14660u;
    o->reinit_us = 300000u;
    static char host[64];
    for (int k = 1; k < argc; k++) {
        const char *a = argv[k];
        const char *v = (k + 1 < argc) ? argv[k + 1] : NULL;
        if (strcmp(a, "--provision") == 0) {
            o->provision = true;
            continue;
        }
        if (v == NULL) {
            return false;
        }
        k++;
        if (strcmp(a, "--fc") == 0) {
            const char *colon = strchr(v, ':');
            if (colon == NULL || (size_t)(colon - v) >= sizeof host) {
                return false;
            }
            memcpy(host, v, (size_t)(colon - v));
            host[colon - v] = '\0';
            o->fc_host = host;
            o->fc_port = (uint16_t)atoi(colon + 1);
        } else if (strcmp(a, "--udp") == 0) {
            o->udp_port = (uint16_t)atoi(v);
        } else if (strcmp(a, "--log") == 0) {
            o->log_path = v;
        } else if (strcmp(a, "--flash") == 0) {
            o->flash_path = v;
        } else if (strcmp(a, "--reinit-ms") == 0) {
            o->reinit_us = (uint32_t)atoi(v) * 1000u;
        } else if (strcmp(a, "--set") == 0 && o->n_sets < MAX_SETS) {
            o->sets[o->n_sets++] = v;
        } else {
            return false;
        }
    }
    return true;
}

static bool apply_set(params_t *p, const char *nv)
{
    char name[48];
    const char *eq = strchr(nv, '=');
    if (eq == NULL || (size_t)(eq - nv) >= sizeof name) {
        return false;
    }
    memcpy(name, nv, (size_t)(eq - nv));
    name[eq - nv] = '\0';
    const param_id_t id = params_find(name);
    const param_def_t *d = params_def(id);
    if (d == NULL) {
        fprintf(stderr, "tof_sil: unknown parameter %s\n", name);
        return false;
    }
    params_status_t st;
    if (d->type == PARAM_TYPE_F32) {
        st = params_set_f32(p, id, strtof(eq + 1, NULL));
    } else {
        st = params_set_u32(p, id, (uint32_t)strtoul(eq + 1, NULL, 0));
    }
    if (st != PARAMS_OK) {
        fprintf(stderr, "tof_sil: %s rejected (%d)\n", nv, (int)st);
        return false;
    }
    return true;
}

static uint32_t params_crc(const params_t *p)
{
    return crc32_compute((const uint8_t *)p->v, sizeof p->v);
}

/* ---- log ---- */

static void log_text(const char *s)
{
    fputc('"', g_log);
    for (; *s != '\0'; s++) {
        const unsigned char c = (unsigned char)*s;
        if (c == '"' || c == '\\') {
            fputc('\\', g_log);
            fputc((int)c, g_log);
        } else if (c < 0x20u || c > 0x7Eu) {
            fprintf(g_log, "\\u%04x", c);
        } else {
            fputc((int)c, g_log);
        }
    }
    fputc('"', g_log);
}

static void log_start(params_load_result_t load)
{
    static const char *const k_load[] = {"A", "B", "defaults"};
    const uint32_t t_us = bsp_time_us();
    const uint64_t wall_us = bsp_wall_us();
    fprintf(g_log, "{\"k\":\"start\",\"t\":%u,\"wall_us\":%llu,\"ver\":\"%s\",\"git\":\"%s\",\"load\":\"%s\",\"params_crc\":\"%08x\",\"params\":{",
            t_us, (unsigned long long)wall_us, tof_version_string(), tof_version_git_hash(), k_load[load],
            params_crc(&g_p));
    for (int k = 0; k < (int)PARAM_COUNT; k++) {
        const param_def_t *d = params_def((param_id_t)k);
        if (d->type == PARAM_TYPE_F32) {
            fprintf(g_log, "%s\"%s\":%.6g", (k > 0) ? "," : "", d->name, (double)params_get_f32(&g_p, (param_id_t)k));
        } else {
            fprintf(g_log, "%s\"%s\":%u", (k > 0) ? "," : "", d->name, params_get_u32(&g_p, (param_id_t)k));
        }
    }
    fprintf(g_log, "}}\n");
}

static void log_map(const sched_map_log_t *m)
{
    fprintf(g_log,
            "{\"k\":\"map\",\"t\":%u,\"seq\":%u,\"t1\":%u,\"t2\":%u,\"t3\":%u,\"t4\":%u,\"t5\":%u,\"sent\":%d,"
            "\"ph\":%u,\"ms\":%u,\"v\":%u,\"att\":%d,\"pitch\":%.4f,\"roll\":%.4f,\"mode\":%u,\"arm\":%d,\"f\":%u,\"d\":[",
            bsp_time_us(), m->seq, m->t1_us, m->t2_us, m->t3_us, m->t4_us, m->t5_us, m->sent ? 1 : 0, (unsigned)m->phase,
            (unsigned)m->map_state, (unsigned)m->view, m->att_valid ? 1 : 0, (double)m->pitch_rad, (double)m->roll_rad, m->fc_mode,
            m->fc_armed ? 1 : 0, (unsigned)m->faults);
    for (uint8_t k = 0; k < TOF_NUM_SECTORS; k++) {
        fprintf(g_log, "%s%u", (k > 0u) ? "," : "", (unsigned)m->distances[k]);
    }
    fprintf(g_log, "]}\n");
}

static void log_event(const ev_log_t *e)
{
    fprintf(g_log, "{\"k\":\"ev\",\"t\":%u,\"kind\":%u,\"a\":%u,\"b\":%u}\n", e->t_us, (unsigned)e->kind, (unsigned)e->a,
            (unsigned)e->b);
}

static void log_status(const sim_sensor_t sim[TOF_NUM_SENSORS], uint32_t now_us)
{
    const sched_t *s = &g_s;
    float rf = 0.0f;
    const bool rf_ok = mavrx_rf_valid(&s->rx, now_us, &rf);
    fprintf(g_log,
            "{\"k\":\"st\",\"t\":%u,\"h\":[%u,%u,%u],\"map\":%u,\"ph\":%u,\"f\":%u,\"link\":%d,\"arm\":%d,\"mode\":%u,"
            "\"ls\":%u,\"att\":%d,\"rf\":%d,\"rf_m\":%.3f,\"od\":%u,\"skip_rate\":%u,\"skip_lost\":%u,\"txovf\":%u,"
            "\"rx_msgs\":%u,\"crc_err\":%u,\"frames\":[%u,%u,%u],\"gaps\":[%u,%u,%u],\"reinits\":[%u,%u,%u],"
            "\"hold_sends\":%u,\"episodes\":%u}\n",
            now_us, (unsigned)tof_core_sensor_health(&s->core, 0), (unsigned)tof_core_sensor_health(&s->core, 1),
            (unsigned)tof_core_sensor_health(&s->core, 2), (unsigned)tof_core_map_state(&s->core),
            (unsigned)s->phase.phase, (unsigned)s->faults,
            mavrx_hb_valid(&s->rx, now_us) ? 1 : 0, s->rx.armed ? 1 : 0, s->rx.custom_mode, (unsigned)s->rx.landed_state,
            mavrx_att_valid(&s->rx, now_us) ? 1 : 0, rf_ok ? 1 : 0, (double)rf, s->stats.od_sent,
            s->stats.od_skipped_rate, s->stats.od_skipped_lost, s->stats.tx_overflows, s->rx.stats.msgs,
            s->rx.stats.crc_errors, s->core.stats[0].frames, s->core.stats[1].frames, s->core.stats[2].frames,
            s->core.stats[0].gaps, s->core.stats[1].gaps, s->core.stats[2].gaps, sim[0].reinits, sim[1].reinits,
            sim[2].reinits, s->fault.sends_total, s->fault.episodes_total);
}

/* Every packet the module puts on the wire, decoded back (OBSTACLE_DISTANCE is in the map record). */
static mavlink_message_t g_tx_buf;
static mavlink_status_t g_tx_status;
static mavlink_message_t g_tx_msg;
static mavlink_status_t g_tx_msg_status;

static void log_tx_bytes(const uint8_t *data, size_t len, uint32_t now_us)
{
    for (size_t k = 0; k < len; k++) {
        if (mavlink_frame_char_buffer(&g_tx_buf, &g_tx_status, data[k], &g_tx_msg, &g_tx_msg_status) !=
            MAVLINK_FRAMING_OK) {
            continue;
        }
        const mavlink_message_t *m = &g_tx_msg;
        switch (m->msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:
            fprintf(g_log, "{\"k\":\"tx\",\"t\":%u,\"id\":0,\"st\":%u}\n", now_us,
                    (unsigned)mavlink_msg_heartbeat_get_system_status(m));
            break;
        case MAVLINK_MSG_ID_STATUSTEXT: {
            char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN + 1];
            memset(text, 0, sizeof text);
            mavlink_msg_statustext_get_text(m, text);
            fprintf(g_log, "{\"k\":\"tx\",\"t\":%u,\"id\":253,\"sev\":%u,\"text\":", now_us,
                    (unsigned)mavlink_msg_statustext_get_severity(m));
            log_text(text);
            fprintf(g_log, "}\n");
            break;
        }
        case MAVLINK_MSG_ID_COMMAND_LONG: {
            mavlink_command_long_t c;
            mavlink_msg_command_long_decode(m, &c);
            fprintf(g_log,
                    "{\"k\":\"tx\",\"t\":%u,\"id\":76,\"cmd\":%u,\"tsys\":%u,\"tcomp\":%u,\"conf\":%u,\"p1\":%.6g,"
                    "\"p2\":%.6g}\n",
                    now_us, (unsigned)c.command, (unsigned)c.target_system, (unsigned)c.target_component,
                    (unsigned)c.confirmation, (double)c.param1,
                    (double)c.param2);
            break;
        }
        case MAVLINK_MSG_ID_COMMAND_ACK:
            fprintf(g_log, "{\"k\":\"tx\",\"t\":%u,\"id\":77,\"cmd\":%u,\"res\":%u}\n", now_us,
                    (unsigned)mavlink_msg_command_ack_get_command(m),
                    (unsigned)mavlink_msg_command_ack_get_result(m));
            break;
        case MAVLINK_MSG_ID_OBSTACLE_DISTANCE:
            break;
        default:
            fprintf(g_log, "{\"k\":\"tx\",\"t\":%u,\"id\":%u}\n", now_us, (unsigned)m->msgid);
            break;
        }
    }
}

/* ---- sensor frames ---- */

static uint32_t rd_u32(const uint8_t *b)
{
    return (uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
}

static uint16_t rd_u16(const uint8_t *b)
{
    return (uint16_t)((uint16_t)b[0] | (uint16_t)((uint16_t)b[1] << 8));
}

static bool decode_frame(const uint8_t *b, size_t n, tof_frame_t *f)
{
    if (n < SIL_FRAME_HDR_SIZE) {
        return false;
    }
    memset(f, 0, sizeof *f);
    f->sensor_id = b[5];
    f->res = b[6];
    f->freq_hz = b[7];
    f->frame_no = rd_u32(&b[8]);
    const uint8_t nz = b[12];
    if (nz > TOF_MAX_ZONES || n < SIL_FRAME_HDR_SIZE + (size_t)nz * SIL_ZONE_SIZE) {
        return false;
    }
    for (uint8_t z = 0; z < nz; z++) {
        const uint8_t *zb = &b[SIL_FRAME_HDR_SIZE + (size_t)z * SIL_ZONE_SIZE];
        tof_zone_t *zone = &f->zone[z];
        zone->nb_target_detected = zb[0];
        zone->ambient_per_spad = rd_u32(&zb[1]);
        for (uint8_t t = 0; t < SIL_TARGETS && t < TOF_TARGETS_PER_ZONE; t++) {
            const uint8_t *tb = &zb[5u + t * SIL_TARGET_SIZE];
            zone->target[t].distance_mm = (int16_t)rd_u16(&tb[0]);
            zone->target[t].range_sigma_mm = rd_u16(&tb[2]);
            zone->target[t].signal_per_spad = rd_u32(&tb[4]);
            zone->target[t].target_status = tb[8];
        }
    }
    return true;
}

typedef struct {
    uint32_t rx_mute_until_us;
    uint32_t tx_mute_until_us;
    bool rx_muted;
    bool tx_muted;
} faults_t;

static bool until_active(bool on, uint32_t until_us, uint32_t now_us)
{
    return on && (int32_t)(until_us - now_us) > 0;
}

static void handle_udp(sim_sensor_t sim[TOF_NUM_SENSORS], faults_t *fl, uint32_t now_us)
{
    static uint8_t b[SIL_MAX_DATAGRAM];
    static tof_frame_t f;
    size_t n;
    while ((n = bsp_udp_read(b, sizeof b)) >= SIL_HDR_SIZE) {
        if (rd_u32(b) != SIL_MAGIC) {
            continue;
        }
        switch (b[4]) {
        case SIL_PKT_FRAME:
            if (decode_frame(b, n, &f) && f.sensor_id < TOF_NUM_SENSORS) {
                if (sim[f.sensor_id].reinit_busy || !sim[f.sensor_id].powered) {
                    sim[f.sensor_id].frames_dropped++;
                    break;
                }
                f.t_ready_us = now_us;
                (void)sched_push_frame(&g_s, &f, now_us);
            }
            break;
        case SIL_PKT_SENSOR:
            if (n >= SIL_HDR_SIZE + 3u && b[5] < TOF_NUM_SENSORS) {
                sim[b[5]].powered = b[6] != 0u;
                for (uint8_t k = 0; k < b[7]; k++) {
                    tof_core_sensor_error(&g_s.core, b[5], now_us);
                }
                fprintf(g_log, "{\"k\":\"sim\",\"t\":%u,\"sensor\":%u,\"powered\":%u,\"spi_errors\":%u}\n", now_us,
                        (unsigned)b[5], (unsigned)b[6], (unsigned)b[7]);
            }
            break;
        case SIL_PKT_CONTROL:
            if (n >= SIL_HDR_SIZE + 5u) {
                const uint32_t arg = rd_u32(&b[6]);
                fprintf(g_log, "{\"k\":\"sim\",\"t\":%u,\"cmd\":%u,\"arg\":%u}\n", now_us, (unsigned)b[5], arg);
                if (b[5] == SIL_CMD_RX_MUTE) {
                    fl->rx_muted = true;
                    fl->rx_mute_until_us = now_us + arg * 1000u;
                } else if (b[5] == SIL_CMD_TX_MUTE) {
                    fl->tx_muted = true;
                    fl->tx_mute_until_us = now_us + arg * 1000u;
                } else if (b[5] == SIL_CMD_INTERNAL) {
                    sched_set_internal_fault(&g_s, arg != 0u);
                } else if (b[5] == SIL_CMD_QUIT) {
                    g_quit = 1;
                } else {
                    /* unknown command: ignored */
                }
            }
            break;
        default:
            break;
        }
    }
}

/* SNS-11 handshake as the driver would do it: a reinit takes reinit_us and succeeds if the sensor
 * is powered; frames are not produced meanwhile. */
static void emulate_driver(sim_sensor_t sim[TOF_NUM_SENSORS], uint32_t reinit_us, uint32_t now_us)
{
    for (uint8_t k = 0; k < TOF_NUM_SENSORS; k++) {
        if (!sim[k].reinit_busy && tof_core_take_reinit_request(&g_s.core, k)) {
            sim[k].reinit_busy = true;
            sim[k].t_reinit_end_us = now_us + reinit_us;
            sim[k].reinits++;
        }
        if (sim[k].reinit_busy && (int32_t)(now_us - sim[k].t_reinit_end_us) >= 0) {
            sim[k].reinit_busy = false;
            tof_core_reinit_done(&g_s.core, k, sim[k].powered, now_us);
            fprintf(g_log, "{\"k\":\"reinit\",\"t\":%u,\"sensor\":%u,\"ok\":%d}\n", now_us, (unsigned)k,
                    sim[k].powered ? 1 : 0);
        }
    }
}

int main(int argc, char **argv)
{
    options_t o;
    if (!parse_args(argc, argv, &o)) {
        usage();
        return 2;
    }
    bsp_sim_init();
    g_log = stdout;
    if (o.log_path != NULL) {
        g_log = fopen(o.log_path, "w");
        if (g_log == NULL) {
            perror(o.log_path);
            return 1;
        }
    }
    setvbuf(g_log, NULL, _IOLBF, 1 << 16);

    /* SW-07: parameters from the A/B slots; both invalid -> safe defaults and an event. */
    static params_flash_t flash;
    static params_store_t store;
    bsp_flash_init(&flash, o.flash_path);
    const params_load_result_t load = params_store_load(&store, &flash, &g_p);
    for (int k = 0; k < o.n_sets; k++) {
        if (!apply_set(&g_p, o.sets[k])) {
            return 2;
        }
    }
    if (params_validate(&g_p) != PARAMS_OK) {
        fprintf(stderr, "tof_sil: parameter set is not valid\n");
        return 2;
    }
    if (o.provision) {
        const params_status_t st = params_store_save(&store, &g_p, false);
        fprintf(stderr, "tof_sil: provision %s, params CRC %08x\n", (st == PARAMS_OK) ? "ok" : "FAILED",
                params_crc(&g_p));
        return (st == PARAMS_OK) ? 0 : 1;
    }

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);
    signal(SIGPIPE, SIG_IGN);

    bsp_uart_config(o.fc_host, o.fc_port);
    if (!bsp_udp_open(o.udp_port)) {
        fprintf(stderr, "tof_sil: cannot bind UDP %u\n", o.udp_port);
        return 1;
    }
    const uint32_t baud = params_get_u32(&g_p, PARAM_UART_BAUD);
    const uint32_t t0 = bsp_time_us();
    sched_init(&g_s, &g_p, load == PARAMS_LOADED_DEFAULTS, t0);
    log_start(load);
    fprintf(stderr, "tof_sil %s (%s): FC %s:%u, frames UDP %u\n", tof_version_string(), tof_version_git_hash(),
            o.fc_host, o.fc_port, o.udp_port);

    sim_sensor_t sim[TOF_NUM_SENSORS];
    memset(sim, 0, sizeof sim);
    for (uint8_t k = 0; k < TOF_NUM_SENSORS; k++) {
        sim[k].powered = true;
    }
    faults_t fl;
    memset(&fl, 0, sizeof fl);
    static uint8_t rx[4096];
    static uint8_t dma[DMA_CHUNK];
    uint32_t dma_len = 0u;
    uint32_t t_dma_end_us = 0u;
    uint32_t t_status_us = t0;

    while (!g_quit) {
        const uint32_t now = bsp_time_us();
        fl.rx_muted = until_active(fl.rx_muted, fl.rx_mute_until_us, now);
        fl.tx_muted = until_active(fl.tx_muted, fl.tx_mute_until_us, now);
        (void)bsp_uart_poll(now);
        size_t n;
        while ((n = bsp_uart_read(rx, sizeof rx)) > 0u) {
            if (!fl.rx_muted) {
                sched_uart_rx(&g_s, rx, (uint32_t)n, now);
            }
        }
        handle_udp(sim, &fl, now);
        emulate_driver(sim, o.reinit_us, now);
        sched_step(&g_s, now);

        /* UART DMA: one transfer at a time, 10 bits per byte at UART_BAUD (ICD-05, MAP-12 t4/t5). */
        if (dma_len > 0u && (int32_t)(now - t_dma_end_us) >= 0) {
            if (!fl.tx_muted) {
                bsp_uart_write(dma, dma_len);
            }
            log_tx_bytes(dma, dma_len, now);
            dma_len = 0u;
            sched_tx_done(&g_s, now);
        }
        if (dma_len == 0u) {
            dma_len = sched_tx_take(&g_s, dma, DMA_CHUNK, now);
            if (dma_len > 0u) {
                t_dma_end_us = now + (uint32_t)(((uint64_t)dma_len * 10u * 1000000u) / baud);
            }
        }

        sched_map_log_t m;
        while (sched_take_map_log(&g_s, &m)) {
            log_map(&m);
        }
        ev_log_t e;
        while (sched_take_event_log(&g_s, &e)) {
            log_event(&e);
        }
        if ((uint32_t)(now - t_status_us) >= STATUS_PERIOD_US) {
            t_status_us = now;
            log_status(sim, now);
        }
        bsp_wait(LOOP_WAIT_US);
    }
    fprintf(g_log, "{\"k\":\"stop\",\"t\":%u}\n", bsp_time_us());
    fclose(g_log);
    return 0;
}
