#include "core/tof_core.h"

#include <string.h>

static void health_array(const tof_core_t *c, sensor_health_t h[TOF_NUM_SENSORS])
{
    for (uint8_t s = 0; s < TOF_NUM_SENSORS; s++) {
        h[s] = c->sh[s].health;
    }
}

/* REQ: SNS-09, SW-02 */
void tof_core_init(tof_core_t *c, const params_t *p, uint32_t now_us)
{
    memset(c, 0, sizeof *c);
    c->params = *p;
    c->freq_hz = (uint8_t)params_get_u32(p, PARAM_SENSOR_FREQ_HZ);
    sh_cfg_from_params(&c->sh_cfg, p);
    for (uint8_t s = 0; s < TOF_NUM_SENSORS; s++) {
        sh_init(&c->sh[s], now_us);
    }
    fusion_init(&c->fusion, p);
    /* SNS-12a: reported once at start (STATUSTEXT "TOF ORIENT NOT CONFIRMED" in E1b). */
    c->orient_not_confirmed_event = !c->fusion.orient_confirmed;
}

/* REQ: SNS-09, SNS-11, MAP-10 */
tof_push_status_t tof_core_push_frame(tof_core_t *c, const tof_frame_t *frame, uint32_t now_us)
{
    const uint8_t s = frame->sensor_id;
    if (s >= TOF_NUM_SENSORS) {
        return TOF_PUSH_BAD_SENSOR;
    }
    if (!c->fusion.staggered && frame->freq_hz == c->freq_hz && frame->res == c->fusion.res &&
        c->fusion.frame[s].fresh) {
        /* A newer frame of the same sensor: fuse the pending set first so no frame is lost and a
         * silent sensor cannot starve the map (MAP-09). */
        sensor_health_t h[TOF_NUM_SENSORS];
        health_array(c, h);
        fusion_run(&c->fusion, h, now_us);
        c->map_pending = true;
    }
    if (frame->freq_hz != c->freq_hz || !fusion_push_frame(&c->fusion, frame)) {
        c->stats[s].rejected++;
        sh_on_error(&c->sh[s], now_us);
        return TOF_PUSH_BAD_MODE;
    }
    if (c->have_frame_no[s] && frame->frame_no != c->last_frame_no[s] + 1u) {
        c->stats[s].gaps++;
    }
    c->have_frame_no[s] = true;
    c->last_frame_no[s] = frame->frame_no;
    c->stats[s].frames++;
    sh_on_frame(&c->sh[s], &c->sh_cfg, c->fusion.frame[s].valid_frac, c->fusion.frame[s].ambient_high,
                now_us);
    if (c->fusion.staggered) {
        /* MAP-09a: every new frame of any sensor triggers a fusion step. */
        sensor_health_t h[TOF_NUM_SENSORS];
        health_array(c, h);
        fusion_run_staggered(&c->fusion, h, s, now_us);
        c->map_pending = true;
    }
    return TOF_PUSH_OK;
}

/* REQ: SNS-11 */
void tof_core_sensor_error(tof_core_t *c, uint8_t sensor, uint32_t now_us)
{
    if (sensor < TOF_NUM_SENSORS) {
        sh_on_error(&c->sh[sensor], now_us);
    }
}

/* REQ: SNS-11 */
bool tof_core_take_reinit_request(tof_core_t *c, uint8_t sensor)
{
    return sensor < TOF_NUM_SENSORS && sh_take_reinit_request(&c->sh[sensor]);
}

/* REQ: SNS-11 */
void tof_core_reinit_done(tof_core_t *c, uint8_t sensor, bool ok, uint32_t now_us)
{
    if (sensor < TOF_NUM_SENSORS) {
        /* A fresh driver counter starts after reinit. */
        c->have_frame_no[sensor] = false;
        sh_on_reinit_done(&c->sh[sensor], ok, now_us);
    }
}

/* REQ: MAP-09, MAP-09a, MAP-10, MAP-11, SNS-12a */
bool tof_core_update(tof_core_t *c, uint32_t now_us)
{
    for (uint8_t s = 0; s < TOF_NUM_SENSORS; s++) {
        sh_update(&c->sh[s], &c->sh_cfg, now_us);
    }
    sensor_health_t h[TOF_NUM_SENSORS];
    health_array(c, h);
    c->fusion.state = fusion_map_state_cfg(&c->fusion, h);
    bool produced = c->map_pending;
    c->map_pending = false;
    /* A sensor in reinit or FAILED is not waited for (MAP-12 latency); its zones stay INV. */
    sensor_health_t h_wait[TOF_NUM_SENSORS];
    for (uint8_t s = 0; s < TOF_NUM_SENSORS; s++) {
        h_wait[s] = (c->sh[s].sup == SH_RUNNING) ? h[s] : SENSOR_FAILED;
    }
    if (!c->fusion.staggered && fusion_ready(&c->fusion, h_wait, now_us)) {
        fusion_run(&c->fusion, h, now_us);
        produced = true;
    }
    return produced;
}

/* REQ: MAP-10 */
sensor_health_t tof_core_sensor_health(const tof_core_t *c, uint8_t sensor)
{
    return (sensor < TOF_NUM_SENSORS) ? c->sh[sensor].health : SENSOR_FAILED;
}

/* REQ: MAP-11 */
map_state_t tof_core_map_state(const tof_core_t *c)
{
    return c->fusion.state;
}

/* REQ: MAP-05, 5.2 */
void tof_core_export(const tof_core_t *c, uint32_t now_us, uint16_t distances[TOF_NUM_SECTORS])
{
    fusion_export(&c->fusion, now_us, distances);
}
