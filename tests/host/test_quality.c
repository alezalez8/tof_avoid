#include <string.h>

#include "core/params.h"
#include "core/quality.h"
#include "unity.h"

static params_t g_p;
static quality_cfg_t g_cfg;

void setUp(void)
{
    params_set_defaults(&g_p);
    quality_cfg_from_params(&g_cfg, &g_p);
}

void tearDown(void) {}

static tof_target_t target(uint8_t status, uint16_t sigma, uint32_t signal)
{
    tof_target_t t;
    memset(&t, 0, sizeof t);
    t.distance_mm = 1000;
    t.target_status = status;
    t.range_sigma_mm = sigma;
    t.signal_per_spad = signal;
    return t;
}

/* Default VALID_STATUS = {5, 9}. */
void test_MAP03_default_valid_status_5_and_9(void)
{
    for (unsigned s = 0; s <= 255u; s++) {
        const tof_target_t t = target((uint8_t)s, 5u, 10u);
        TEST_ASSERT_EQUAL((s == 5u || s == 9u), quality_target_valid(&g_cfg, &t));
    }
}

void test_MAP03_custom_valid_status_mask(void)
{
    (void)params_set_u32(&g_p, PARAM_VALID_STATUS, 1u << 5);
    quality_cfg_from_params(&g_cfg, &g_p);
    tof_target_t t = target(5u, 5u, 10u);
    TEST_ASSERT_TRUE(quality_target_valid(&g_cfg, &t));
    t.target_status = 9u;
    TEST_ASSERT_FALSE(quality_target_valid(&g_cfg, &t));
    /* Status values above 31 cannot be selected by the mask. */
    (void)params_set_u32(&g_p, PARAM_VALID_STATUS, 0xFFFFFFFFu);
    quality_cfg_from_params(&g_cfg, &g_p);
    t.target_status = 31u;
    TEST_ASSERT_TRUE(quality_target_valid(&g_cfg, &t));
    t.target_status = 32u;
    TEST_ASSERT_FALSE(quality_target_valid(&g_cfg, &t));
    t.target_status = 255u;
    TEST_ASSERT_FALSE(quality_target_valid(&g_cfg, &t));
}

void test_MAP03_sigma_limit_inclusive(void)
{
    (void)params_set_u32(&g_p, PARAM_SIGMA_MAX_MM, 20u);
    quality_cfg_from_params(&g_cfg, &g_p);
    tof_target_t t = target(5u, 20u, 10u);
    TEST_ASSERT_TRUE(quality_target_valid(&g_cfg, &t));
    t.range_sigma_mm = 21u;
    TEST_ASSERT_FALSE(quality_target_valid(&g_cfg, &t));
}

void test_MAP03_default_sigma_has_no_limit(void)
{
    const tof_target_t t = target(5u, 65535u, 10u);
    TEST_ASSERT_TRUE(quality_target_valid(&g_cfg, &t));
}

void test_MAP03_signal_minimum_inclusive(void)
{
    (void)params_set_u32(&g_p, PARAM_SIGNAL_MIN, 100u);
    quality_cfg_from_params(&g_cfg, &g_p);
    tof_target_t t = target(9u, 5u, 100u);
    TEST_ASSERT_TRUE(quality_target_valid(&g_cfg, &t));
    t.signal_per_spad = 99u;
    TEST_ASSERT_FALSE(quality_target_valid(&g_cfg, &t));
}

void test_MAP03_default_signal_min_zero_accepts_zero(void)
{
    const tof_target_t t = target(5u, 5u, 0u);
    TEST_ASSERT_TRUE(quality_target_valid(&g_cfg, &t));
}

/* Up to 2 targets per zone (SNS-06): each target is classified separately. */
void test_MAP03_zone_two_targets_classified_separately(void)
{
    tof_zone_t z;
    memset(&z, 0, sizeof z);
    z.nb_target_detected = 2u;
    z.target[0] = target(4u, 5u, 10u); /* invalid */
    z.target[1] = target(5u, 5u, 10u); /* valid */
    quality_zone_t q;
    quality_zone_eval(&g_cfg, &z, &q);
    TEST_ASSERT_EQUAL_UINT8(2u, q.n_targets);
    TEST_ASSERT_EQUAL_UINT8(1u, q.n_valid);
    TEST_ASSERT_EQUAL_HEX8(0x02u, q.valid_mask);
    TEST_ASSERT_TRUE(q.zone_valid);
}

void test_MAP03_zone_ignores_targets_beyond_two(void)
{
    tof_zone_t z;
    memset(&z, 0, sizeof z);
    z.nb_target_detected = 4u; /* more than configured: only the first two are read */
    z.target[0] = target(5u, 5u, 10u);
    z.target[1] = target(9u, 5u, 10u);
    quality_zone_t q;
    quality_zone_eval(&g_cfg, &z, &q);
    TEST_ASSERT_EQUAL_UINT8(TOF_TARGETS_PER_ZONE, q.n_targets);
    TEST_ASSERT_EQUAL_HEX8(0x03u, q.valid_mask);
}

void test_MAP03_zone_without_targets(void)
{
    tof_zone_t z;
    memset(&z, 0, sizeof z);
    z.target[0] = target(5u, 5u, 10u); /* stale buffer content must be ignored */
    quality_zone_t q;
    quality_zone_eval(&g_cfg, &z, &q);
    TEST_ASSERT_EQUAL_UINT8(0u, q.n_targets);
    TEST_ASSERT_EQUAL_UINT8(0u, q.n_valid);
    TEST_ASSERT_EQUAL_HEX8(0u, q.valid_mask);
    /* MAP-10a: a zone without a target is neither valid nor invalid; it is not counted. */
    TEST_ASSERT_FALSE(q.has_target);
    TEST_ASSERT_FALSE(q.zone_valid);
}

void test_MAP03_zone_with_only_invalid_targets_is_invalid(void)
{
    tof_zone_t z;
    memset(&z, 0, sizeof z);
    z.nb_target_detected = 1u;
    z.target[0] = target(255u, 5u, 10u);
    quality_zone_t q;
    quality_zone_eval(&g_cfg, &z, &q);
    TEST_ASSERT_EQUAL_UINT8(1u, q.n_targets);
    TEST_ASSERT_TRUE(q.has_target);
    TEST_ASSERT_FALSE(q.zone_valid);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_MAP03_default_valid_status_5_and_9);
    RUN_TEST(test_MAP03_custom_valid_status_mask);
    RUN_TEST(test_MAP03_sigma_limit_inclusive);
    RUN_TEST(test_MAP03_default_sigma_has_no_limit);
    RUN_TEST(test_MAP03_signal_minimum_inclusive);
    RUN_TEST(test_MAP03_default_signal_min_zero_accepts_zero);
    RUN_TEST(test_MAP03_zone_two_targets_classified_separately);
    RUN_TEST(test_MAP03_zone_ignores_targets_beyond_two);
    RUN_TEST(test_MAP03_zone_without_targets);
    RUN_TEST(test_MAP03_zone_with_only_invalid_targets_is_invalid);
    return UNITY_END();
}
