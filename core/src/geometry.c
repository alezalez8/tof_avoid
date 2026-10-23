#include "core/geometry.h"

#include <math.h>

/* REQ: MAP-02 (helper) */
float geo_deg_to_rad(float deg)
{
    return deg * GEO_DEG_TO_RAD_F;
}

/* REQ: MAP-02 (helper) */
float geo_wrap_deg_360(float deg)
{
    float r = fmodf(deg, 360.0f);
    if (r < 0.0f) {
        r += 360.0f;
    }
    /* A tiny negative input rounds up to exactly 360 after the addition. */
    if (r >= 360.0f) {
        r = 0.0f;
    }
    return r;
}
