#include "core/geometry.h"
#include "unity.h"

void setUp(void) {}
void tearDown(void) {}

void test_MAP02_deg_to_rad(void)
{
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, geo_deg_to_rad(0.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, GEO_PI_F, geo_deg_to_rad(180.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, -GEO_PI_F / 2.0f, geo_deg_to_rad(-90.0f));
}

void test_MAP02_wrap_keeps_range_0_360(void)
{
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, geo_wrap_deg_360(0.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 45.0f, geo_wrap_deg_360(45.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, geo_wrap_deg_360(360.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 5.0f, geo_wrap_deg_360(725.0f));
}

void test_MAP02_wrap_negative_angles(void)
{
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 355.0f, geo_wrap_deg_360(-5.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 330.0f, geo_wrap_deg_360(-30.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, geo_wrap_deg_360(-360.0f));
}

void test_MAP02_wrap_tiny_negative_is_below_360(void)
{
    const float r = geo_wrap_deg_360(-1e-6f);
    TEST_ASSERT_TRUE(r >= 0.0f);
    TEST_ASSERT_TRUE(r < 360.0f);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_MAP02_deg_to_rad);
    RUN_TEST(test_MAP02_wrap_keeps_range_0_360);
    RUN_TEST(test_MAP02_wrap_negative_angles);
    RUN_TEST(test_MAP02_wrap_tiny_negative_is_below_360);
    return UNITY_END();
}
