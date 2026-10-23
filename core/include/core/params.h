#ifndef CORE_PARAMS_H
#define CORE_PARAMS_H

#include <stdbool.h>
#include <stdint.h>

#include "core/config.h"

/* Module parameters (SW-07): one table (name, type, default, min, max) and A/B slot storage. */

typedef enum {
    PARAM_TYPE_U8,
    PARAM_TYPE_U16,
    PARAM_TYPE_U32,
    PARAM_TYPE_F32
} param_type_t;

/* One stored value; integers use .u, F32 uses .f. Stored as a 32-bit word. */
typedef union {
    uint32_t u;
    float f;
} param_value_t;

typedef struct {
    const char *name;
    param_type_t type;
    param_value_t def;
    param_value_t min;
    param_value_t max;
} param_def_t;

#define X(name, type, def, min, max) PARAM_##name,
typedef enum {
#include "core/params_table.h"
    PARAM_COUNT
} param_id_t;
#undef X

/* GROUND_POLICY values (MAP-07). */
#define PARAM_GROUND_KEEP 0u
#define PARAM_GROUND_DROP 1u

/* FAULT_ACTION values (FLT-03). */
#define PARAM_FAULT_NONE 0u
#define PARAM_FAULT_BRAKE 1u
#define PARAM_FAULT_LOITER 2u

/* FAULT_TRIGGER_MASK bits: fault classes of FLT-01. */
#define PARAM_FAULT_BIT_SENSOR_FAILED (1u << 0)
#define PARAM_FAULT_BIT_SENSOR_DEGRADED (1u << 1)
#define PARAM_FAULT_BIT_MAP_STALE (1u << 2)
#define PARAM_FAULT_BIT_MAP_LOST (1u << 3)
#define PARAM_FAULT_BIT_ATT_ALT_LOST (1u << 4)
#define PARAM_FAULT_BIT_INTERNAL (1u << 5)

/* SCHEDULE values (SNS-07). */
#define PARAM_SCHEDULE_SIMULTANEOUS 0u
#define PARAM_SCHEDULE_STAGGERED 1u

typedef struct {
    param_value_t v[PARAM_COUNT];
} params_t;

typedef enum {
    PARAMS_OK = 0,
    PARAMS_ERR_ID,      /* unknown parameter id */
    PARAMS_ERR_TYPE,    /* integer accessor used on F32 or vice versa */
    PARAMS_ERR_RANGE,   /* value outside [min, max] */
    PARAMS_ERR_INVALID, /* cross-parameter constraint violated */
    PARAMS_ERR_ARMED,   /* save refused: FC is armed */
    PARAMS_ERR_FLASH,   /* flash driver reported an error */
    PARAMS_ERR_VERIFY,  /* read-back after write does not match */
    PARAMS_ERR_SIZE     /* slot too small for the image */
} params_status_t;

/* Table access. Returns NULL for an unknown id. */
const param_def_t *params_def(param_id_t id);

/* Id by exact name, or PARAM_COUNT if not found. */
param_id_t params_find(const char *name);

void params_set_defaults(params_t *p);

/* True if v lies in [min, max] of the parameter (NaN is never in range). */
bool params_in_range(param_id_t id, param_value_t v);

params_status_t params_set_u32(params_t *p, param_id_t id, uint32_t value);
params_status_t params_set_f32(params_t *p, param_id_t id, float value);

/* Unchecked getters: the id must be valid and of the matching type. */
uint32_t params_get_u32(const params_t *p, param_id_t id);
float params_get_f32(const params_t *p, param_id_t id);

/* Whole-set check: every value in range and cross-parameter constraints hold. */
params_status_t params_validate(const params_t *p);

/* ---- A/B slot storage ---- */

#define PARAMS_FORMAT_VERSION 4u /* 2: ORIENT_CONFIRMED, AMBIENT_DEGRADED (TZ 5.6); 3: CAL_TOL_EL_DEG (TZ 5.7);
                                  * 4: EXPOSURE_OFFSET_MS (DEV-12 p. 6) */
#define PARAMS_MAGIC 0x50464F54u /* "TOFP" little-endian */
#define PARAMS_HEADER_SIZE 24u
#define PARAMS_IMAGE_SIZE (PARAMS_HEADER_SIZE + 4u * (uint32_t)PARAM_COUNT)

#define PARAMS_SLOT_A 0u
#define PARAMS_SLOT_B 1u
#define PARAMS_SLOT_NONE 0xFFu

/* Flash abstraction implemented by the BSP (or a mock in tests). Every call returns false on error.
 * A slot is erased as a whole; writes may be split into any number of calls. */
typedef struct {
    void *ctx;
    uint32_t slot_size;
    bool (*erase)(void *ctx, uint8_t slot);
    bool (*write)(void *ctx, uint8_t slot, uint32_t offset, const uint8_t *src, uint32_t len);
    bool (*read)(void *ctx, uint8_t slot, uint32_t offset, uint8_t *dst, uint32_t len);
} params_flash_t;

typedef struct {
    const params_flash_t *flash;
    uint8_t active_slot; /* PARAMS_SLOT_A/B, or PARAMS_SLOT_NONE when defaults are in use */
    uint32_t active_seq;
    bool defaults_event; /* set by load when both slots were invalid */
} params_store_t;

typedef enum {
    PARAMS_LOADED_SLOT_A,
    PARAMS_LOADED_SLOT_B,
    PARAMS_LOADED_DEFAULTS /* both slots invalid: safe defaults, event raised */
} params_load_result_t;

/* Picks the valid slot with the highest sequence number; both invalid -> defaults. */
params_load_result_t params_store_load(params_store_t *st, const params_flash_t *flash, params_t *out);

/* Writes p to the inactive slot with seq + 1 and verifies it by read-back. Only when disarmed. */
params_status_t params_store_save(params_store_t *st, const params_t *p, bool fc_armed);

#endif /* CORE_PARAMS_H */
