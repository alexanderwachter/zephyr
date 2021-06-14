#include <ztest.h>
#include <kernel.h>
#include <mutex.hpp>
#include <clock.hpp>

extern "C"
{
K_THREAD_STACK_DEFINE(stack, 512);
static const size_t stack_size = K_THREAD_STACK_SIZEOF(stack);
static struct k_thread thread;
}

zpp::recursive_mutex global_mutex1;
zpp::recursive_mutex global_mutex2;

void thread_function(void *, void *, void *)
{
    global_mutex1.lock();
    global_mutex2.lock();
    global_mutex1.unlock();
    global_mutex2.unlock();
}

void test_trylock()
{
    global_mutex2.lock();

    k_thread_create(&thread, stack, stack_size, thread_function, nullptr, nullptr, nullptr,
                    CONFIG_MAIN_THREAD_PRIORITY - 1,  0, K_NO_WAIT);

    k_yield();

    bool tryLockRes = global_mutex1.try_lock();
    zassert_false(tryLockRes, "This mutes should not be available");

    global_mutex2.unlock();
    global_mutex1.lock();

    k_thread_join(&thread, K_FOREVER);
}

static constexpr uint32_t TRY_TIME = 500;
void lock_the_mutex(void *mtxArg, void *, void *)
{
    zpp::recursive_timed_mutex *mtx = reinterpret_cast<zpp::recursive_timed_mutex*>(mtxArg);
    mtx->lock();
}

void lock_mutex_in_thread(zpp::recursive_timed_mutex& mtx)
{
    k_thread_create(&thread, stack, stack_size, lock_the_mutex, &mtx, nullptr, nullptr,
                    CONFIG_MAIN_THREAD_PRIORITY - 1,  0, K_NO_WAIT);

    k_yield();
    k_thread_join(&thread, K_FOREVER);
}

void test_try_lock_for()
{
    zpp::recursive_timed_mutex mtx;

    lock_mutex_in_thread(mtx);

    const auto start = k_uptime_get();
    mtx.try_lock_for(std::chrono::milliseconds(TRY_TIME));
    const auto end = k_uptime_get();

    zassert_within(end - start, TRY_TIME, TRY_TIME/10, "Time does not match");
}

void test_try_lock_until()
{
    zpp::recursive_timed_mutex mtx;

    lock_mutex_in_thread(mtx);

    const auto now = zpp::chrono::steady_clock::now();
    const auto abs_timeout = now + std::chrono::milliseconds(TRY_TIME);
    const auto start = k_uptime_get();
    mtx.try_lock_until(abs_timeout);
    const auto end = k_uptime_get();

    zassert_within(end - start, TRY_TIME, TRY_TIME/10, "Time does not match");
}

void test_main(void)
{
    ztest_test_suite(recursive_mutex,
                     ztest_unit_test(test_trylock),
                     ztest_unit_test(test_try_lock_for),
                     ztest_unit_test(test_try_lock_until));

    ztest_run_test_suite(recursive_mutex);
}
