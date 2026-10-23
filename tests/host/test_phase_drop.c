#include "core/phase.h"
#include "unity.h"

/* Built with TOF_ALLOW_GROUND_DROP = 1 (MAP-07a): the policy the human enables after T-22, T-25, T-26. */

void setUp(void) {}
void tearDown(void) {}

/* MAP-07, MAP-17: DROP acts only in FLIGHT and only with a valid height. */
void test_MAP07_drop_only_in_flight_with_height(void)
{
    TEST_ASSERT_FALSE(phase_export_ground(PHASE_FLIGHT, PARAM_GROUND_DROP, true));
    TEST_ASSERT_TRUE(phase_export_ground(PHASE_FLIGHT, PARAM_GROUND_DROP, false));
    TEST_ASSERT_TRUE(phase_export_ground(PHASE_FLIGHT, PARAM_GROUND_KEEP, true));
    TEST_ASSERT_TRUE(phase_export_ground(PHASE_UNKNOWN, PARAM_GROUND_DROP, true));
    TEST_ASSERT_FALSE(phase_export_ground(PHASE_TAKEOFF, PARAM_GROUND_DROP, true));
    TEST_ASSERT_FALSE(phase_export_ground(PHASE_GROUND, PARAM_GROUND_DROP, true));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_MAP07_drop_only_in_flight_with_height);
    return UNITY_END();
}
