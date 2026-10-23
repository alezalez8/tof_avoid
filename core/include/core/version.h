#ifndef CORE_VERSION_H
#define CORE_VERSION_H

#include <stdint.h>

/* Single source of truth for the firmware version; CMake parses these lines. */
#define TOF_VERSION_MAJOR 0
#define TOF_VERSION_MINOR 1
#define TOF_VERSION_PATCH 0

#define TOF_VERSION_STR_(x) #x
#define TOF_VERSION_STR(x) TOF_VERSION_STR_(x)
#define TOF_VERSION_STRING                                                      \
    TOF_VERSION_STR(TOF_VERSION_MAJOR) "." TOF_VERSION_STR(TOF_VERSION_MINOR) "." \
    TOF_VERSION_STR(TOF_VERSION_PATCH)

/* Semver string, e.g. "0.1.0". */
const char *tof_version_string(void);

/* Short git hash captured at configure time, or "unknown". */
const char *tof_version_git_hash(void);

/* Packed version: major << 16 | minor << 8 | patch. */
uint32_t tof_version_number(void);

#endif /* CORE_VERSION_H */
