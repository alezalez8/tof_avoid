#include "core/sensor_health.h"
#include "unity.h"

#define MS 1000u
#define PERIOD_US 66666u /* 15 Hz */

static sh_cfg_t g_cfg;
static sh_t g_s;

void setUp(void)
{
    params_t p;
    params_set_defaults(&p);
    sh_cfg_from_params(&g_cfg, &p);
    sh_init(&g_s, 0u);
}

void tearDown(void) {}

/* Feeds good frames every period from t0 to t1 inclusive. */
static uint32_t frames(uint32_t t0, uint32_t t1, float frac)
{
    uint32_t t = t0;
    for (; t <= t1; t += PERIOD_US) {
        sh_on_frame(&g_s, &g_cfg, frac, false, t);
        sh_update(&g_s, &g_cfg, t);
    }
    return t - PERIOD_US;
}

/* Drives the sensor into REINIT by three errors and takes the request. */
static void start_reinit(uint32_t t)
{
    sh_on_error(&g_s, t);
    sh_on_error(&g_s, t);
    sh_on_error(&g_s, t);
    TEST_ASSERT_TRUE(sh_take_reinit_request(&g_s));
}

void test_SNS11_config_from_params(void)
{
    TEST_ASSERT_UINT32_WITHIN(1u, PERIOD_US, g_cfg.period_us);
    TEST_ASSERT_EQUAL_FLOAT(0.5f, g_cfg.valid_frac_min);
    TEST_ASSERT_EQUAL_UINT32(300u * MS, g_cfg.degraded_time_us);
    TEST_ASSERT_EQUAL_INT(SH_RUNNING, g_s.sup);
    TEST_ASSERT_EQUAL_INT(SENSOR_OK, g_s.health);
}

/* SNS-11: 3 errors in a row -> reinit of this sensor. */
void test_SNS11_three_consecutive_errors_request_reinit(void)
{
    sh_on_error(&g_s, 10u * MS);
    sh_on_error(&g_s, 20u * MS);
    TEST_ASSERT_FALSE(sh_take_reinit_request(&g_s));
    sh_on_error(&g_s, 30u * MS);
    TEST_ASSERT_EQUAL_INT(SH_REINIT, g_s.sup);
    TEST_ASSERT_TRUE(sh_take_reinit_request(&g_s));
    TEST_ASSERT_FALSE(sh_take_reinit_request(&g_s)); /* taken once */
    TEST_ASSERT_EQUAL_UINT32(1u, g_s.reinit_total);
}

void test_SNS11_frame_between_errors_resets_count(void)
{
    sh_on_error(&g_s, 10u * MS);
    sh_on_error(&g_s, 20u * MS);
    sh_on_frame(&g_s, &g_cfg, 1.0f, false, 30u * MS);
    sh_on_error(&g_s, 40u * MS);
    sh_on_error(&g_s, 50u * MS);
    TEST_ASSERT_FALSE(sh_take_reinit_request(&g_s));
    TEST_ASSERT_EQUAL_INT(SH_RUNNING, g_s.sup);
}

/* SNS-11: no frame for longer than 3 periods -> reinit; detection <= 300 ms (T-07). */
void test_SNS11_no_frame_for_3_periods_requests_reinit(void)
{
    sh_on_frame(&g_s, &g_cfg, 1.0f, false, 0u);
    sh_update(&g_s, &g_cfg, 3u * PERIOD_US);
    TEST_ASSERT_FALSE(sh_take_reinit_request(&g_s));
    sh_update(&g_s, &g_cfg, 3u * PERIOD_US + 1u);
    TEST_ASSERT_TRUE(sh_take_reinit_request(&g_s));
    TEST_ASSERT_TRUE(3u * PERIOD_US + 1u <= 300u * MS);
}

/* Errors while a reinit is already pending do not request another one. */
void test_SNS11_errors_during_reinit_are_ignored(void)
{
    start_reinit(0u);
    sh_on_error(&g_s, 10u * MS);
    sh_on_error(&g_s, 20u * MS);
    sh_on_error(&g_s, 30u * MS);
    TEST_ASSERT_FALSE(sh_take_reinit_request(&g_s));
    /* The no-frame check is suspended while reinit runs. */
    sh_update(&g_s, &g_cfg, 500u * MS);
    TEST_ASSERT_FALSE(sh_take_reinit_request(&g_s));
}

void test_SNS11_successful_reinit_and_frame_recover(void)
{
    start_reinit(0u);
    sh_on_reinit_done(&g_s, true, 400u * MS);
    TEST_ASSERT_EQUAL_INT(SH_RUNNING, g_s.sup);
    sh_on_frame(&g_s, &g_cfg, 1.0f, false, 450u * MS);
    TEST_ASSERT_EQUAL_UINT8(0u, g_s.consec_reinit_fails);
    TEST_ASSERT_FALSE(g_s.reinit_unconfirmed);
    sh_update(&g_s, &g_cfg, 450u * MS);
    TEST_ASSERT_EQUAL_INT(SENSOR_OK, g_s.health);
}

/* SNS-11: 5 failed reinits in a row -> FAILED. */
void test_SNS11_five_failed_reinits_give_failed(void)
{
    start_reinit(0u);
    for (unsigned k = 1; k < SH_REINIT_FAILS_TO_FAILED; k++) {
        sh_on_reinit_done(&g_s, false, k * 100u * MS);
        TEST_ASSERT_EQUAL_INT(SH_REINIT, g_s.sup);
        TEST_ASSERT_TRUE(sh_take_reinit_request(&g_s)); /* immediate new attempt */
    }
    sh_on_reinit_done(&g_s, false, 500u * MS);
    TEST_ASSERT_EQUAL_INT(SH_FAILED, g_s.sup);
    TEST_ASSERT_FALSE(sh_take_reinit_request(&g_s));
    sh_update(&g_s, &g_cfg, 500u * MS);
    TEST_ASSERT_EQUAL_INT(SENSOR_FAILED, g_s.health);
}

/* A reinit reported OK but followed by no frame counts as a failure. */
void test_SNS11_reinit_ok_without_frame_counts_as_failure(void)
{
    uint32_t t = 0u;
    start_reinit(t);
    for (unsigned k = 0; k < SH_REINIT_FAILS_TO_FAILED; k++) {
        sh_on_reinit_done(&g_s, true, t);
        TEST_ASSERT_EQUAL_INT(SH_RUNNING, g_s.sup);
        t += 3u * PERIOD_US + 1u;
        sh_update(&g_s, &g_cfg, t); /* no frame: triggers again */
        if (k + 1u < SH_REINIT_FAILS_TO_FAILED) {
            TEST_ASSERT_TRUE(sh_take_reinit_request(&g_s));
        }
    }
    TEST_ASSERT_EQUAL_INT(SH_FAILED, g_s.sup);
}

/* A reinit not reported within 1 s counts as failed and is requested again. */
void test_SNS11_reinit_timeout_counts_as_failure(void)
{
    start_reinit(0u);
    sh_update(&g_s, &g_cfg, SH_REINIT_TIMEOUT_US);
    TEST_ASSERT_FALSE(sh_take_reinit_request(&g_s));
    sh_update(&g_s, &g_cfg, SH_REINIT_TIMEOUT_US + 1u);
    TEST_ASSERT_EQUAL_UINT8(1u, g_s.consec_reinit_fails);
    TEST_ASSERT_TRUE(sh_take_reinit_request(&g_s));
}

/* SNS-11: in FAILED a retry every 10 s. */
void test_SNS11_failed_retries_every_10_s(void)
{
    start_reinit(0u);
    for (unsigned k = 0; k < SH_REINIT_FAILS_TO_FAILED; k++) {
        sh_on_reinit_done(&g_s, false, 1000u * MS);
        (void)sh_take_reinit_request(&g_s);
    }
    TEST_ASSERT_EQUAL_INT(SH_FAILED, g_s.sup);
    sh_update(&g_s, &g_cfg, 1000u * MS + SH_FAILED_RETRY_US - 1u);
    TEST_ASSERT_FALSE(sh_take_reinit_request(&g_s));
    sh_update(&g_s, &g_cfg, 1000u * MS + SH_FAILED_RETRY_US);
    TEST_ASSERT_TRUE(sh_take_reinit_request(&g_s));
    TEST_ASSERT_EQUAL_INT(SH_REINIT, g_s.sup);
    /* The retry fails: back to FAILED, next retry 10 s after this one started. */
    sh_on_reinit_done(&g_s, false, 11200u * MS);
    TEST_ASSERT_EQUAL_INT(SH_FAILED, g_s.sup);
    sh_update(&g_s, &g_cfg, 20999u * MS);
    TEST_ASSERT_FALSE(sh_take_reinit_request(&g_s));
    sh_update(&g_s, &g_cfg, 21000u * MS);
    TEST_ASSERT_TRUE(sh_take_reinit_request(&g_s));
}

void test_SNS11_retry_from_failed_recovers_with_frame(void)
{
    start_reinit(0u);
    for (unsigned k = 0; k < SH_REINIT_FAILS_TO_FAILED; k++) {
        sh_on_reinit_done(&g_s, false, 0u);
        (void)sh_take_reinit_request(&g_s);
    }
    sh_update(&g_s, &g_cfg, SH_FAILED_RETRY_US);
    TEST_ASSERT_TRUE(sh_take_reinit_request(&g_s));
    sh_on_reinit_done(&g_s, true, SH_FAILED_RETRY_US + 300u * MS);
    sh_on_frame(&g_s, &g_cfg, 1.0f, false, SH_FAILED_RETRY_US + 350u * MS);
    sh_update(&g_s, &g_cfg, SH_FAILED_RETRY_US + 350u * MS);
    TEST_ASSERT_EQUAL_INT(SH_RUNNING, g_s.sup);
    TEST_ASSERT_EQUAL_INT(SENSOR_OK, g_s.health);
}

/* Retry from FAILED reported OK, but no frame: straight back to FAILED. */
void test_SNS11_retry_ok_without_frame_returns_to_failed(void)
{
    start_reinit(0u);
    for (unsigned k = 0; k < SH_REINIT_FAILS_TO_FAILED; k++) {
        sh_on_reinit_done(&g_s, false, 0u);
        (void)sh_take_reinit_request(&g_s);
    }
    sh_update(&g_s, &g_cfg, SH_FAILED_RETRY_US);
    TEST_ASSERT_TRUE(sh_take_reinit_request(&g_s));
    sh_on_reinit_done(&g_s, true, SH_FAILED_RETRY_US);
    sh_update(&g_s, &g_cfg, SH_FAILED_RETRY_US + 3u * PERIOD_US + 1u);
    TEST_ASSERT_EQUAL_INT(SH_FAILED, g_s.sup);
    TEST_ASSERT_FALSE(sh_take_reinit_request(&g_s));
}

/* SNS-11: only the failing sensor is reinitialised; the others keep working. */
void test_SNS11_other_sensors_unaffected(void)
{
    sh_t s[3];
    for (unsigned k = 0; k < 3u; k++) {
        sh_init(&s[k], 0u);
    }
    for (uint32_t t = 0; t <= 1000u * MS; t += PERIOD_US) {
        sh_on_frame(&s[0], &g_cfg, 1.0f, false, t);
        sh_on_frame(&s[2], &g_cfg, 1.0f, false, t);
        for (unsigned k = 0; k < 3u; k++) {
            sh_update(&s[k], &g_cfg, t);
        }
    }
    TEST_ASSERT_TRUE(sh_take_reinit_request(&s[1]));
    TEST_ASSERT_FALSE(sh_take_reinit_request(&s[0]));
    TEST_ASSERT_FALSE(sh_take_reinit_request(&s[2]));
    TEST_ASSERT_EQUAL_INT(SENSOR_OK, s[0].health);
    TEST_ASSERT_EQUAL_INT(SENSOR_OK, s[2].health);
}

/* MAP-10: DEGRADED when the valid fraction is below VALID_FRAC_MIN for longer than DEGRADED_TIME_MS. */
void test_MAP10_low_valid_fraction_longer_than_300ms_degraded(void)
{
    (void)frames(0u, 100u * MS, 1.0f);
    const uint32_t t_low = 100u * MS + PERIOD_US;
    sh_on_frame(&g_s, &g_cfg, 0.4f, false, t_low);
    sh_update(&g_s, &g_cfg, t_low);
    sh_on_frame(&g_s, &g_cfg, 0.4f, false, t_low + 300u * MS);
    sh_update(&g_s, &g_cfg, t_low + 300u * MS);
    TEST_ASSERT_EQUAL_INT(SENSOR_OK, g_s.health); /* exactly 300 ms */
    sh_update(&g_s, &g_cfg, t_low + 300u * MS + 1u);
    TEST_ASSERT_EQUAL_INT(SENSOR_DEGRADED, g_s.health);
    /* Recovery: one frame with enough valid zones. */
    sh_on_frame(&g_s, &g_cfg, 0.9f, false, t_low + 350u * MS);
    sh_update(&g_s, &g_cfg, t_low + 350u * MS);
    TEST_ASSERT_EQUAL_INT(SENSOR_OK, g_s.health);
}

void test_MAP10_fraction_equal_to_min_is_ok(void)
{
    (void)frames(0u, 1000u * MS, 0.5f);
    TEST_ASSERT_EQUAL_INT(SENSOR_OK, g_s.health);
}

void test_MAP10_short_low_fraction_stays_ok(void)
{
    (void)frames(0u, 200u * MS, 0.1f);
    (void)frames(201u * MS, 1000u * MS, 1.0f);
    TEST_ASSERT_EQUAL_INT(SENSOR_OK, g_s.health);
}

/* No frames count as no valid zones: DEGRADED 300 ms after the last frame, FAILED after SNS-11. */
void test_MAP10_missing_frames_degraded_then_failed(void)
{
    sh_on_frame(&g_s, &g_cfg, 1.0f, false, 0u);
    sh_update(&g_s, &g_cfg, 250u * MS); /* reinit triggered at > 200 ms */
    TEST_ASSERT_TRUE(sh_take_reinit_request(&g_s));
    TEST_ASSERT_EQUAL_INT(SENSOR_OK, g_s.health);
    sh_update(&g_s, &g_cfg, 301u * MS);
    TEST_ASSERT_EQUAL_INT(SENSOR_DEGRADED, g_s.health);
    for (unsigned k = 0; k < SH_REINIT_FAILS_TO_FAILED; k++) {
        sh_on_reinit_done(&g_s, false, 400u * MS);
        (void)sh_take_reinit_request(&g_s);
    }
    sh_update(&g_s, &g_cfg, 400u * MS);
    TEST_ASSERT_EQUAL_INT(SENSOR_FAILED, g_s.health);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_SNS11_config_from_params);
    RUN_TEST(test_SNS11_three_consecutive_errors_request_reinit);
    RUN_TEST(test_SNS11_frame_between_errors_resets_count);
    RUN_TEST(test_SNS11_no_frame_for_3_periods_requests_reinit);
    RUN_TEST(test_SNS11_errors_during_reinit_are_ignored);
    RUN_TEST(test_SNS11_successful_reinit_and_frame_recover);
    RUN_TEST(test_SNS11_five_failed_reinits_give_failed);
    RUN_TEST(test_SNS11_reinit_ok_without_frame_counts_as_failure);
    RUN_TEST(test_SNS11_reinit_timeout_counts_as_failure);
    RUN_TEST(test_SNS11_failed_retries_every_10_s);
    RUN_TEST(test_SNS11_retry_from_failed_recovers_with_frame);
    RUN_TEST(test_SNS11_retry_ok_without_frame_returns_to_failed);
    RUN_TEST(test_SNS11_other_sensors_unaffected);
    RUN_TEST(test_MAP10_low_valid_fraction_longer_than_300ms_degraded);
    RUN_TEST(test_MAP10_fraction_equal_to_min_is_ok);
    RUN_TEST(test_MAP10_short_low_fraction_stays_ok);
    RUN_TEST(test_MAP10_missing_frames_degraded_then_failed);
    return UNITY_END();
}
