#ifndef CORE_CONFIG_H
#define CORE_CONFIG_H

/* Build-time constants of the core. Runtime values live in the parameter table (params.h). */

/* Number of ToF sensors: L, C, R. */
#define TOF_NUM_SENSORS 3u

/* Largest supported zone matrix side (8x8) and zone count. */
#define TOF_MAX_RES 8u
#define TOF_MAX_ZONES (TOF_MAX_RES * TOF_MAX_RES)

/* Targets per zone (SNS-06): 2 by default; 1 is a build option for experiments only. */
#ifndef TOF_TARGETS_PER_ZONE
#define TOF_TARGETS_PER_ZONE 2u
#endif

/* OBSTACLE_DISTANCE grid: 72 sectors of 5 degrees (MAP-05). */
#define TOF_NUM_SECTORS 72u
#define TOF_SECTOR_DEG 5.0f

/* Horizontal FoV of one VL53L8CX matrix side, degrees (5.1). */
#define TOF_SENSOR_FOV_DEG 45.0f

/* Sensor indices. */
#define TOF_SENSOR_L 0u
#define TOF_SENSOR_C 1u
#define TOF_SENSOR_R 2u

#endif /* CORE_CONFIG_H */
