#include <ztest.h>
#include <kernel.h>
#include <condition_variable.hpp>
#include <clock.hpp>
#include <mutex.hpp>

extern "C" {
K_THREAD_STACK_DEFINE(stack1, 512);
K_THREAD_STACK_DEFINE(stack2, 512);
static const size_t stack1_size = K_THREAD_STACK_SIZEOF(stack1);
static const size_t stack2_size = K_THREAD_STACK_SIZEOF(stack2);

static struct k_thread thread1;
static struct k_thread thread2;
}

Mcu::recursive_mutex mtx;

void thread_notify_one(void *cVarArg, void *, void *)
{
    Mcu::condition_variable *cVar = reinterpret_cast<Mcu::condition_variable*>(cVarArg);

    cVar->notify_one();
}

void thread_notify_all(void *cVarArg, void *, void *)
{
    Mcu::condition_variable *cVar = reinterpret_cast<Mcu::condition_variable*>(cVarArg);

    cVar->notify_all();
}

void thread_lock_and_notify_one(void *cVarLockArg, void *cVarNotifyArg, void *)
{
    Mcu::condition_variable *cVarLock = reinterpret_cast<Mcu::condition_variable*>(cVarLockArg);
    Mcu::condition_variable *cVarNotify = reinterpret_cast<Mcu::condition_variable*>(cVarNotifyArg);

    std::unique_lock<Mcu::recursive_mutex> lock(mtx);
    cVarLock->wait(lock);
    cVarNotify->notify_one();
}

void test_notify_one()
{
    Mcu::condition_variable cVar1;
    Mcu::condition_variable cVar2;

    k_thread_create(&thread1, stack1, stack1_size, thread_notify_one, &cVar1, nullptr, nullptr,
                    CONFIG_MAIN_THREAD_PRIORITY + 1,  0, K_NO_WAIT);

    std::unique_lock<Mcu::recursive_mutex> lock(mtx);
    cVar1.wait(lock);

    k_thread_join(&thread1, K_FOREVER);
}

void test_notify_all()
{
    Mcu::condition_variable cVar1;
    Mcu::condition_variable cVar2;

    k_thread_create(&thread1, stack1, stack1_size, thread_notify_all, &cVar1, nullptr, nullptr,
                    CONFIG_MAIN_THREAD_PRIORITY + 1,  0, K_NO_WAIT);

    k_thread_create(&thread2, stack2, stack2_size, thread_lock_and_notify_one,
                    &cVar1, &cVar2, nullptr, CONFIG_MAIN_THREAD_PRIORITY - 1,  0, K_NO_WAIT);

    std::unique_lock<Mcu::recursive_mutex> lock(mtx);
    cVar1.wait(lock);
    cVar2.wait(lock);

    k_thread_join(&thread1, K_FOREVER);
    k_thread_join(&thread2, K_FOREVER);
}


static constexpr uint32_t TRY_TIME = 500;

void test_try_wait_for()
{
    Mcu::condition_variable cVar;
    std::unique_lock<Mcu::recursive_mutex> lock(mtx);

    const auto start = k_uptime_get();
    const auto res = cVar.wait_for(lock, std::chrono::milliseconds(TRY_TIME));
    const auto end = k_uptime_get();

    zassert_equal(res, Mcu::cv_status::timeout ,"cond. var must not lock");
    zassert_within(end - start, TRY_TIME, TRY_TIME/10, "Time does not match");
}

void test_try_wait_until()
{
    Mcu::condition_variable cVar;
    std::unique_lock<Mcu::recursive_mutex> lock(mtx);

    const auto now = Mcu::chrono::steady_clock::now();
    const auto abs_timeout = now + std::chrono::milliseconds(TRY_TIME);
    const auto start = k_uptime_get();
    const auto res = cVar.wait_until(lock, abs_timeout);
    const auto end = k_uptime_get();

    zassert_equal(res, Mcu::cv_status::timeout ,"cond. var must not lock");
    zassert_within(end - start, TRY_TIME, TRY_TIME/10, "Time does not match");
}


void test_main(void)
{
    ztest_test_suite(condition_variable,
                     ztest_unit_test(test_notify_one),
                     ztest_unit_test(test_notify_all),
                     ztest_unit_test(test_try_wait_for),
                     ztest_unit_test(test_try_wait_until));

    ztest_run_test_suite(condition_variable);
}
