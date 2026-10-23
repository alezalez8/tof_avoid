#include "core/version.h"

#ifndef TOF_GIT_HASH
#define TOF_GIT_HASH "unknown"
#endif

/* REQ: SW-10 */
const char *tof_version_string(void)
{
    return TOF_VERSION_STRING;
}

/* REQ: SW-10 */
const char *tof_version_git_hash(void)
{
    return TOF_GIT_HASH;
}

/* REQ: SW-10 */
uint32_t tof_version_number(void)
{
    return ((uint32_t)TOF_VERSION_MAJOR << 16) | ((uint32_t)TOF_VERSION_MINOR << 8) |
           (uint32_t)TOF_VERSION_PATCH;
}
