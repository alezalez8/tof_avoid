#include "core/params.h"

#include <string.h>

#include "core/crc32.h"

#define PV_U(x) {.u = (x)}
#define PV_F(x) {.f = (x)}
#define X(name, type, def, min, max) {#name, PARAM_TYPE_##type, def, min, max},
static const param_def_t k_table[PARAM_COUNT] = {
#include "core/params_table.h"
};
#undef X

/* Header layout (little-endian): magic u32, format u16, length u16, seq u32, schema u32,
 * reserved u32, crc u32. CRC-32 covers bytes [0, 20) and the payload. */
#define HDR_MAGIC 0u
#define HDR_FORMAT 4u
#define HDR_LENGTH 6u
#define HDR_SEQ 8u
#define HDR_SCHEMA 12u
#define HDR_RESERVED 16u
#define HDR_CRC 20u
#define PAYLOAD_SIZE (4u * (uint32_t)PARAM_COUNT)

static uint32_t type_limit(param_type_t t)
{
    switch (t) {
    case PARAM_TYPE_U8:
        return 0xFFu;
    case PARAM_TYPE_U16:
        return 0xFFFFu;
    default:
        return 0xFFFFFFFFu;
    }
}

/* REQ: SW-07 */
const param_def_t *params_def(param_id_t id)
{
    if ((unsigned)id >= (unsigned)PARAM_COUNT) {
        return NULL;
    }
    return &k_table[id];
}

/* REQ: SW-07 */
param_id_t params_find(const char *name)
{
    for (unsigned i = 0; i < (unsigned)PARAM_COUNT; i++) {
        if (strcmp(k_table[i].name, name) == 0) {
            return (param_id_t)i;
        }
    }
    return PARAM_COUNT;
}

/* REQ: SW-07 */
void params_set_defaults(params_t *p)
{
    for (unsigned i = 0; i < (unsigned)PARAM_COUNT; i++) {
        p->v[i] = k_table[i].def;
    }
}

/* REQ: SW-07 */
bool params_in_range(param_id_t id, param_value_t v)
{
    const param_def_t *d = params_def(id);
    if (d == NULL) {
        return false;
    }
    if (d->type == PARAM_TYPE_F32) {
        /* NaN fails both comparisons. */
        return v.f >= d->min.f && v.f <= d->max.f;
    }
    return v.u <= type_limit(d->type) && v.u >= d->min.u && v.u <= d->max.u;
}

static params_status_t set_value(params_t *p, param_id_t id, param_value_t v, bool is_float)
{
    const param_def_t *d = params_def(id);
    if (d == NULL) {
        return PARAMS_ERR_ID;
    }
    if ((d->type == PARAM_TYPE_F32) != is_float) {
        return PARAMS_ERR_TYPE;
    }
    if (!params_in_range(id, v)) {
        return PARAMS_ERR_RANGE;
    }
    p->v[id] = v;
    return PARAMS_OK;
}

/* REQ: SW-07 */
params_status_t params_set_u32(params_t *p, param_id_t id, uint32_t value)
{
    const param_value_t v = {.u = value};
    return set_value(p, id, v, false);
}

/* REQ: SW-07 */
params_status_t params_set_f32(params_t *p, param_id_t id, float value)
{
    const param_value_t v = {.f = value};
    return set_value(p, id, v, true);
}

/* REQ: SW-07 */
uint32_t params_get_u32(const params_t *p, param_id_t id)
{
    return p->v[id].u;
}

/* REQ: SW-07 */
float params_get_f32(const params_t *p, param_id_t id)
{
    return p->v[id].f;
}

/* REQ: SW-07, SNS-05 */
params_status_t params_validate(const params_t *p)
{
    for (unsigned i = 0; i < (unsigned)PARAM_COUNT; i++) {
        if (!params_in_range((param_id_t)i, p->v[i])) {
            return PARAMS_ERR_RANGE;
        }
    }
    const uint32_t res = p->v[PARAM_SENSOR_RES].u;
    const uint32_t freq = p->v[PARAM_SENSOR_FREQ_HZ].u;
    if (res != 4u && res != 8u) {
        return PARAMS_ERR_INVALID;
    }
    /* SNS-05: 4x4 up to 30 Hz, 8x8 up to 15 Hz. */
    if (res == 8u && freq > 15u) {
        return PARAMS_ERR_INVALID;
    }
    /* MAP-04: at least one row of the active matrix must stay in the map. */
    if ((p->v[PARAM_ROW_MASK].u & ((1u << res) - 1u)) == 0u) {
        return PARAMS_ERR_INVALID;
    }
    if (p->v[PARAM_MIN_DIST_CM].u >= p->v[PARAM_MAX_DIST_CM].u) {
        return PARAMS_ERR_INVALID;
    }
    if (p->v[PARAM_CRIT_CM].u >= p->v[PARAM_WARN_CM].u) {
        return PARAMS_ERR_INVALID;
    }
    return PARAMS_OK;
}

/* ---- Serialization ---- */

static void put_u16(uint8_t *b, uint16_t v)
{
    b[0] = (uint8_t)v;
    b[1] = (uint8_t)(v >> 8);
}

static void put_u32(uint8_t *b, uint32_t v)
{
    b[0] = (uint8_t)v;
    b[1] = (uint8_t)(v >> 8);
    b[2] = (uint8_t)(v >> 16);
    b[3] = (uint8_t)(v >> 24);
}

static uint16_t get_u16(const uint8_t *b)
{
    return (uint16_t)((uint16_t)b[0] | ((uint16_t)b[1] << 8));
}

static uint32_t get_u32(const uint8_t *b)
{
    return (uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
}

/* CRC of names and types: an image written by firmware with a different table is rejected
 * even if PARAMS_FORMAT_VERSION was not bumped. */
static uint32_t schema_crc(void)
{
    uint32_t c = CRC32_INIT;
    for (unsigned i = 0; i < (unsigned)PARAM_COUNT; i++) {
        const uint8_t type = (uint8_t)k_table[i].type;
        c = crc32_update(c, (const uint8_t *)k_table[i].name, strlen(k_table[i].name));
        c = crc32_update(c, &type, 1u);
    }
    return crc32_final(c);
}

/* REQ: SW-07 */
static uint32_t image_crc(const uint8_t *img)
{
    const uint32_t c = crc32_update(CRC32_INIT, img, HDR_CRC);
    return crc32_final(crc32_update(c, &img[PARAMS_HEADER_SIZE], PAYLOAD_SIZE));
}

/* REQ: SW-07 */
static void serialize(const params_t *p, uint32_t seq, uint8_t *img)
{
    put_u32(&img[HDR_MAGIC], PARAMS_MAGIC);
    put_u16(&img[HDR_FORMAT], (uint16_t)PARAMS_FORMAT_VERSION);
    put_u16(&img[HDR_LENGTH], (uint16_t)PAYLOAD_SIZE);
    put_u32(&img[HDR_SEQ], seq);
    put_u32(&img[HDR_SCHEMA], schema_crc());
    put_u32(&img[HDR_RESERVED], 0u);
    for (unsigned i = 0; i < (unsigned)PARAM_COUNT; i++) {
        put_u32(&img[PARAMS_HEADER_SIZE + 4u * i], p->v[i].u);
    }
    put_u32(&img[HDR_CRC], image_crc(img));
}

/* Reads and checks one slot. On success fills out and seq. */
/* REQ: SW-07 */
static bool load_slot(const params_flash_t *flash, uint8_t slot, params_t *out, uint32_t *seq)
{
    uint8_t img[PARAMS_IMAGE_SIZE];
    if (flash->slot_size < PARAMS_IMAGE_SIZE ||
        !flash->read(flash->ctx, slot, 0u, img, PARAMS_IMAGE_SIZE)) {
        return false;
    }
    if (get_u32(&img[HDR_MAGIC]) != PARAMS_MAGIC ||
        get_u16(&img[HDR_FORMAT]) != PARAMS_FORMAT_VERSION ||
        get_u16(&img[HDR_LENGTH]) != PAYLOAD_SIZE || get_u32(&img[HDR_SCHEMA]) != schema_crc() ||
        get_u32(&img[HDR_CRC]) != image_crc(img)) {
        return false;
    }
    params_t tmp;
    for (unsigned i = 0; i < (unsigned)PARAM_COUNT; i++) {
        tmp.v[i].u = get_u32(&img[PARAMS_HEADER_SIZE + 4u * i]);
    }
    if (params_validate(&tmp) != PARAMS_OK) {
        return false;
    }
    *out = tmp;
    *seq = get_u32(&img[HDR_SEQ]);
    return true;
}

/* REQ: SW-07 */
params_load_result_t params_store_load(params_store_t *st, const params_flash_t *flash, params_t *out)
{
    params_t pa;
    params_t pb;
    uint32_t seq_a = 0u;
    uint32_t seq_b = 0u;
    const bool ok_a = load_slot(flash, PARAMS_SLOT_A, &pa, &seq_a);
    const bool ok_b = load_slot(flash, PARAMS_SLOT_B, &pb, &seq_b);

    st->flash = flash;
    st->defaults_event = false;
    if (ok_b && (!ok_a || seq_b > seq_a)) {
        *out = pb;
        st->active_slot = PARAMS_SLOT_B;
        st->active_seq = seq_b;
        return PARAMS_LOADED_SLOT_B;
    }
    if (ok_a) {
        *out = pa;
        st->active_slot = PARAMS_SLOT_A;
        st->active_seq = seq_a;
        return PARAMS_LOADED_SLOT_A;
    }
    /* Both invalid: safe defaults (FAULT_ACTION = BRAKE, GROUND_POLICY = KEEP) and an event. */
    params_set_defaults(out);
    st->active_slot = PARAMS_SLOT_NONE;
    st->active_seq = 0u;
    st->defaults_event = true;
    return PARAMS_LOADED_DEFAULTS;
}

/* REQ: SW-07 */
params_status_t params_store_save(params_store_t *st, const params_t *p, bool fc_armed)
{
    if (fc_armed) {
        return PARAMS_ERR_ARMED;
    }
    const params_status_t vs = params_validate(p);
    if (vs != PARAMS_OK) {
        return vs;
    }
    const params_flash_t *flash = st->flash;
    if (flash->slot_size < PARAMS_IMAGE_SIZE) {
        return PARAMS_ERR_SIZE;
    }
    /* Re-scan both slots: a read error at boot must not let an older slot with a higher sequence
     * win over this save on the next boot (T-27). */
    params_t tmp;
    uint32_t seq_a = 0u;
    uint32_t seq_b = 0u;
    const bool ok_a = load_slot(flash, PARAMS_SLOT_A, &tmp, &seq_a);
    const bool ok_b = load_slot(flash, PARAMS_SLOT_B, &tmp, &seq_b);
    uint8_t newest = st->active_slot;
    uint32_t base = st->active_seq;
    if (ok_a && (!ok_b || seq_a >= seq_b)) {
        newest = PARAMS_SLOT_A;
        base = (seq_a > base) ? seq_a : base;
    } else if (ok_b) {
        newest = PARAMS_SLOT_B;
        base = (seq_b > base) ? seq_b : base;
    }
    const uint8_t target = (newest == PARAMS_SLOT_A) ? PARAMS_SLOT_B : PARAMS_SLOT_A;
    const uint32_t seq = base + 1u;

    uint8_t img[PARAMS_IMAGE_SIZE];
    serialize(p, seq, img);

    /* Payload first, header last: an interrupted write never leaves a valid header. */
    if (!flash->erase(flash->ctx, target) ||
        !flash->write(flash->ctx, target, PARAMS_HEADER_SIZE, &img[PARAMS_HEADER_SIZE],
                      PAYLOAD_SIZE) ||
        !flash->write(flash->ctx, target, 0u, img, PARAMS_HEADER_SIZE)) {
        return PARAMS_ERR_FLASH;
    }
    uint8_t back[PARAMS_IMAGE_SIZE];
    if (!flash->read(flash->ctx, target, 0u, back, PARAMS_IMAGE_SIZE)) {
        return PARAMS_ERR_FLASH;
    }
    if (memcmp(back, img, PARAMS_IMAGE_SIZE) != 0) {
        return PARAMS_ERR_VERIFY;
    }
    st->active_slot = target;
    st->active_seq = seq;
    return PARAMS_OK;
}
