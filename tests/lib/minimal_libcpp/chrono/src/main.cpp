#include <ztest.h>
#include <kernel.h>
#include <clock.hpp>

static constexpr uint32_t SLEEP_TIME = 1000000;

void test_chrono_time_span()
{
    const auto start = zpp::chrono::steady_clock::now();
    k_usleep(SLEEP_TIME);
    const auto end = zpp::chrono::steady_clock::now();
    const auto diff = end - start;
    zassert_within(std::chrono::microseconds(diff).count(), SLEEP_TIME, SLEEP_TIME/10, "Time does not match");
}

void test_main(void)
{
    ztest_test_suite(steady_clock,
					 ztest_unit_test(test_chrono_time_span));

	ztest_run_test_suite(steady_clock);
}
