/* Parameter table of the module (SW-07, TZ section 15). Included by params.h and params.c only.
 *
 * X(NAME, TYPE, DEFAULT, MIN, MAX)
 *   TYPE: U8, U16, U32 (unsigned integer) or F32 (float).
 *   DEFAULT/MIN/MAX: PV_U(x) for integer types, PV_F(x) for F32.
 *
 * Defaults are the TZ start values; min/max are agent-chosen physical bounds (docs/deviations.md, DEV-07).
 * Build-time parameters (BOARD, SPI_TOPOLOGY, targets per zone) are not here: see CMake and config.h.
 * Adding, removing or reordering entries requires bumping PARAMS_FORMAT_VERSION.
 */

#ifndef TOF_PARAM_LCD_ENABLE_DEFAULT
#define TOF_PARAM_LCD_ENABLE_DEFAULT 1u /* 1 in DEBUG; the FLIGHT build may override */
#endif
#ifndef TOF_PARAM_LOG_LEVEL_DEFAULT
#define TOF_PARAM_LOG_LEVEL_DEFAULT 1u /* raw zones in DEBUG; FLIGHT build sets 0 */
#endif

/* Sensors (SNS-05, SNS-07, SNS-12) */
X(SENSOR_RES,            U8,  PV_U(4u),     PV_U(4u),    PV_U(8u))
X(SENSOR_FREQ_HZ,        U8,  PV_U(15u),    PV_U(1u),    PV_U(30u))
X(SCHEDULE,              U8,  PV_U(0u),     PV_U(0u),    PV_U(1u))
X(ORIENT_TRANSFORM_L,    U8,  PV_U(0u),     PV_U(0u),    PV_U(7u))
X(ORIENT_TRANSFORM_C,    U8,  PV_U(0u),     PV_U(0u),    PV_U(7u))
X(ORIENT_TRANSFORM_R,    U8,  PV_U(0u),     PV_U(0u),    PV_U(7u))

/* Mounting (MAP-02, HW-07) */
X(MOUNT_YAW_DEG_L,       F32, PV_F(-30.0f), PV_F(-180.0f), PV_F(180.0f))
X(MOUNT_YAW_DEG_C,       F32, PV_F(0.0f),   PV_F(-180.0f), PV_F(180.0f))
X(MOUNT_YAW_DEG_R,       F32, PV_F(30.0f),  PV_F(-180.0f), PV_F(180.0f))
X(MOUNT_PITCH_DEG_L,     F32, PV_F(0.0f),   PV_F(-15.0f),  PV_F(15.0f))
X(MOUNT_PITCH_DEG_C,     F32, PV_F(0.0f),   PV_F(-15.0f),  PV_F(15.0f))
X(MOUNT_PITCH_DEG_R,     F32, PV_F(0.0f),   PV_F(-15.0f),  PV_F(15.0f))
X(MOUNT_ROLL_DEG_L,      F32, PV_F(0.0f),   PV_F(-15.0f),  PV_F(15.0f))
X(MOUNT_ROLL_DEG_C,      F32, PV_F(0.0f),   PV_F(-15.0f),  PV_F(15.0f))
X(MOUNT_ROLL_DEG_R,      F32, PV_F(0.0f),   PV_F(-15.0f),  PV_F(15.0f))
X(MOUNT_X_M_L,           F32, PV_F(0.0f),   PV_F(-1.0f),   PV_F(1.0f))
X(MOUNT_Y_M_L,           F32, PV_F(0.0f),   PV_F(-1.0f),   PV_F(1.0f))
X(MOUNT_Z_M_L,           F32, PV_F(0.0f),   PV_F(-1.0f),   PV_F(1.0f))
X(MOUNT_X_M_C,           F32, PV_F(0.0f),   PV_F(-1.0f),   PV_F(1.0f))
X(MOUNT_Y_M_C,           F32, PV_F(0.0f),   PV_F(-1.0f),   PV_F(1.0f))
X(MOUNT_Z_M_C,           F32, PV_F(0.0f),   PV_F(-1.0f),   PV_F(1.0f))
X(MOUNT_X_M_R,           F32, PV_F(0.0f),   PV_F(-1.0f),   PV_F(1.0f))
X(MOUNT_Y_M_R,           F32, PV_F(0.0f),   PV_F(-1.0f),   PV_F(1.0f))
X(MOUNT_Z_M_R,           F32, PV_F(0.0f),   PV_F(-1.0f),   PV_F(1.0f))
X(RF_X_M,                F32, PV_F(0.0f),   PV_F(-1.0f),   PV_F(1.0f))
X(RF_Y_M,                F32, PV_F(0.0f),   PV_F(-1.0f),   PV_F(1.0f))
X(RF_Z_M,                F32, PV_F(0.0f),   PV_F(-1.0f),   PV_F(1.0f))
X(TOF_HEIGHT_GROUND_M,   F32, PV_F(0.20f),  PV_F(0.05f),   PV_F(1.0f))

/* Target quality (MAP-03); VALID_STATUS is a bit mask of target_status values: {5, 9} */
X(VALID_STATUS,          U32, PV_U(0x00000220u), PV_U(0u), PV_U(0xFFFFFFFFu))
X(SIGMA_MAX_MM,          U16, PV_U(65535u), PV_U(1u),    PV_U(65535u))  /* 65535 = no limit */
X(SIGNAL_MIN,            U32, PV_U(0u),     PV_U(0u),    PV_U(1000000u))
X(AMBIENT_CLEAR_MAX,     U32, PV_U(0u),     PV_U(0u),    PV_U(1000000u)) /* 0 = CLEAR_MEASURED off */

/* Map (MAP-04, MAP-05, MAP-09, MAP-10, MAP-13) */
X(ROW_MASK,              U8,  PV_U(0xFFu),  PV_U(1u),    PV_U(0xFFu))   /* bit i = row i, i = 0 top */
X(CAL_TOL_DEG,           F32, PV_F(2.0f),   PV_F(0.0f),  PV_F(10.0f))
X(CLEAR_CONFIRM_FRAMES,  U8,  PV_U(3u),     PV_U(1u),    PV_U(50u))
X(DIST_RELAX_FRAMES,     U8,  PV_U(2u),     PV_U(1u),    PV_U(50u))
X(HOLD_MAX_MS,           U16, PV_U(500u),   PV_U(0u),    PV_U(5000u))
X(FUSION_WINDOW_MS,      U16, PV_U(0u),     PV_U(0u),    PV_U(1000u))   /* 0 = 1.5 frame periods */
X(VALID_DATA_MAX_AGE_MS, U16, PV_U(150u),   PV_U(10u),   PV_U(2000u))
X(VALID_FRAC_MIN,        F32, PV_F(0.5f),   PV_F(0.0f),  PV_F(1.0f))
X(DEGRADED_TIME_MS,      U16, PV_U(300u),   PV_U(0u),    PV_U(10000u))

/* Ground and flight phases (MAP-07, MAP-14, MAP-16) */
X(GROUND_POLICY,         U8,  PV_U(0u),     PV_U(0u),    PV_U(1u))      /* 0 = KEEP, 1 = DROP */
X(GROUND_TOL_M,          F32, PV_F(0.15f),  PV_F(0.0f),  PV_F(1.0f))
X(PHASE_H_FLIGHT_M,      F32, PV_F(1.5f),   PV_F(0.3f),  PV_F(7.0f))
X(PHASE_H_HYST_M,        F32, PV_F(0.5f),   PV_F(0.0f),  PV_F(3.0f))

/* MAVLink (ICD) */
X(MIN_DIST_CM,           U16, PV_U(4u),     PV_U(1u),    PV_U(600u))
X(MAX_DIST_CM,           U16, PV_U(300u),   PV_U(10u),   PV_U(600u))
X(MAV_DISTANCE_RATE_HZ,  U8,  PV_U(10u),    PV_U(1u),    PV_U(30u))
X(UART_BAUD,             U32, PV_U(115200u), PV_U(9600u), PV_U(921600u))
X(MAV_SYSID,             U8,  PV_U(0u),     PV_U(0u),    PV_U(255u))    /* 0 = from FC HEARTBEAT */
X(MAV_COMPID,            U8,  PV_U(196u),   PV_U(1u),    PV_U(255u))

/* Faults (FLT) */
X(FAULT_ACTION,          U8,  PV_U(1u),     PV_U(0u),    PV_U(2u))      /* 0 NONE, 1 BRAKE, 2 LOITER */
X(FAULT_TRIGGER_MASK,    U8,  PV_U(0x0Du),  PV_U(0u),    PV_U(0x3Fu))   /* PARAM_FAULT_BIT_* */
X(FAULT_ON_REBOOT,       U8,  PV_U(1u),     PV_U(0u),    PV_U(1u))
X(FAULT_WALL,            U8,  PV_U(0u),     PV_U(0u),    PV_U(1u))

/* Diagnostics (DIA) */
X(WARN_CM,               U16, PV_U(250u),   PV_U(10u),   PV_U(600u))
X(CRIT_CM,               U16, PV_U(120u),   PV_U(10u),   PV_U(600u))
X(WARN_HYST_CM,          U16, PV_U(30u),    PV_U(0u),    PV_U(200u))
X(LCD_ENABLE,            U8,  PV_U(TOF_PARAM_LCD_ENABLE_DEFAULT), PV_U(0u), PV_U(1u))
X(LCD_RATE_HZ,           U8,  PV_U(5u),     PV_U(1u),    PV_U(20u))
X(NAMED_VALUE_ENABLE,    U8,  PV_U(0u),     PV_U(0u),    PV_U(1u))
X(LOG_LEVEL,             U8,  PV_U(TOF_PARAM_LOG_LEVEL_DEFAULT), PV_U(0u), PV_U(1u))
