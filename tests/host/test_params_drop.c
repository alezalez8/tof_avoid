/* Built with TOF_ALLOW_GROUND_DROP = 1 (MAP-07a): DROP becomes an accepted value. */
#include "core/params.h"
#include "unity.h"

void setUp(void) {}
void tearDown(void) {}

void test_MAP07a_drop_accepted_with_build_flag(void)
{
    params_t p;
    params_set_defaults(&p);
    TEST_ASSERT_EQUAL_UINT32(1u, TOF_ALLOW_GROUND_DROP);
    /* The default stays KEEP even when DROP is allowed (MAP-07, CLAUDE.md section 7). */
    TEST_ASSERT_EQUAL_UINT32(PARAM_GROUND_KEEP, params_get_u32(&p, PARAM_GROUND_POLICY));
    TEST_ASSERT_EQUAL_INT(PARAMS_OK, params_set_u32(&p, PARAM_GROUND_POLICY, PARAM_GROUND_DROP));
    TEST_ASSERT_EQUAL_INT(PARAMS_OK, params_validate(&p));
    TEST_ASSERT_EQUAL_INT(PARAMS_ERR_RANGE, params_set_u32(&p, PARAM_GROUND_POLICY, 2u));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_MAP07a_drop_accepted_with_build_flag);
    return UNITY_END();
}
