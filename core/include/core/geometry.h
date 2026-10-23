#ifndef CORE_GEOMETRY_H
#define CORE_GEOMETRY_H

/* E0 stub: angle helpers only. Frames and zone geometry (MAP-01, MAP-02) come in E1. */

#define GEO_PI_F 3.14159265f
#define GEO_DEG_TO_RAD_F (GEO_PI_F / 180.0f)

/* Degrees to radians, single precision. */
float geo_deg_to_rad(float deg);

/* Wraps an angle to [0, 360) degrees. */
float geo_wrap_deg_360(float deg);

#endif /* CORE_GEOMETRY_H */
