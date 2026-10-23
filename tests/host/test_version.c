#include <stdio.h>
#include <string.h>

#include "core/version.h"
#include "unity.h"

void setUp(void) {}
void tearDown(void) {}

void test_SW10_version_string_matches_macros(void)
{
    char expected[32];
    (void)snprintf(expected, sizeof expected, "%d.%d.%d", TOF_VERSION_MAJOR, TOF_VERSION_MINOR,
                   TOF_VERSION_PATCH);
    TEST_ASSERT_EQUAL_STRING(expected, tof_version_string());
}

void test_SW10_version_number_is_packed(void)
{
    const uint32_t expected = ((uint32_t)TOF_VERSION_MAJOR << 16) |
                              ((uint32_t)TOF_VERSION_MINOR << 8) | (uint32_t)TOF_VERSION_PATCH;
    TEST_ASSERT_EQUAL_HEX32(expected, tof_version_number());
}

void test_SW10_git_hash_is_present(void)
{
    const char *hash = tof_version_git_hash();
    TEST_ASSERT_NOT_NULL(hash);
    TEST_ASSERT_TRUE(strlen(hash) > 0u);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_SW10_version_string_matches_macros);
    RUN_TEST(test_SW10_version_number_is_packed);
    RUN_TEST(test_SW10_git_hash_is_present);
    return UNITY_END();
}
