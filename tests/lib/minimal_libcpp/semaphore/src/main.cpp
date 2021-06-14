#include <ztest.h>
#include <kernel.h>
#include <semaphore.hpp>
#include <clock.hpp>

extern "C"
{
K_THREAD_STACK_DEFINE(stack, 512);
static const size_t stack_size = K_THREAD_STACK_SIZEOF(stack);
static struct k_thread thread;
}

zpp::binary_semaphore global_bin_semaphore1;
zpp::binary_semaphore global_bin_semaphore2;

zpp::counting_semaphore<2> global_counting_semaphore1;
zpp::counting_semaphore<2> global_counting_semaphore2;

void thread_function_counting(void *, void *, void *)
{
    global_counting_semaphore2.acquire();
    global_counting_semaphore2.acquire();
    global_counting_semaphore1.release(2);

}

void test_lock_releas_mutual_counting()
{
    bool tryLockRes = global_counting_semaphore1.try_acquire();
    zassert_false(tryLockRes, "This semaphore should not be available");

    k_thread_create(&thread, stack, stack_size, thread_function_counting, nullptr, nullptr, nullptr,
                    CONFIG_MAIN_THREAD_PRIORITY - 1,  0, K_NO_WAIT);

    k_yield();

    global_counting_semaphore2.release();
    global_counting_semaphore2.release();
    global_counting_semaphore1.acquire();
    global_counting_semaphore1.acquire();

    tryLockRes = global_counting_semaphore1.try_acquire();
    zassert_false(tryLockRes, "This semaphore should not be available");

    k_thread_join(&thread, K_FOREVER);
}

void thread_function_binary(void *, void *, void *)
{
    global_bin_semaphore2.acquire();
    global_bin_semaphore1.release();
}

void test_lock_releas_mutual_binary()
{

    bool tryLockRes = global_bin_semaphore1.try_acquire();
    zassert_false(tryLockRes, "This semaphore should not be available");

    k_thread_create(&thread, stack, stack_size, thread_function_binary, nullptr, nullptr, nullptr,
                    CONFIG_MAIN_THREAD_PRIORITY - 1,  0, K_NO_WAIT);

    k_yield();

    global_bin_semaphore2.release();
    global_bin_semaphore1.acquire();

    k_thread_join(&thread, K_FOREVER);
}

static constexpr uint32_t TRY_TIME = 500;

void test_try_lock_for()
{
    zpp::binary_semaphore sem;

    const auto start = k_uptime_get();
    sem.try_acquire_for(std::chrono::milliseconds(TRY_TIME));
    const auto end = k_uptime_get();

    zassert_within(end - start, TRY_TIME, TRY_TIME/10, "Time does not match");
}

void test_try_lock_until()
{
    zpp::binary_semaphore sem;

    const auto now = zpp::chrono::steady_clock::now();
    const auto abs_timeout = now + std::chrono::milliseconds(TRY_TIME);
    const auto start = k_uptime_get();
    sem.try_acquire_until(abs_timeout);
    const auto end = k_uptime_get();

    zassert_within(end - start, TRY_TIME, TRY_TIME/10, "Time does not match");
}

void test_main(void)
{
	ztest_test_suite(semaphore,
                     ztest_unit_test(test_lock_releas_mutual_binary),
                     ztest_unit_test(test_lock_releas_mutual_counting),
                     ztest_unit_test(test_try_lock_for),
                     ztest_unit_test(test_try_lock_until)
                     );

	ztest_run_test_suite(semaphore);
}
