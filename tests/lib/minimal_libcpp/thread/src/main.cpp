#include <ztest.h>
#include <kernel.h>
#include <thread.hpp>
#include <semaphore.hpp>

MCU_CREATE_STACK(my_stack, 512)

class thread_container
{
    using DelegateT = zpp::thread::DelegateT;
    static constexpr int prio = CONFIG_MAIN_THREAD_PRIORITY - 1;

public:
    thread_container(const zpp::ThreadStackC stack) noexcept
    : m_thread(DelegateT::create<thread_container, &thread_container::thread_function>(this), stack, prio)
    , m_started_sem()
    , m_block_sem()
    {}

    void execute() noexcept
    {
        m_thread.start();
        zpp::this_thread::yield();
    }

    bool thread_executed() noexcept
    {
        return m_started_sem.try_acquire();
    }

    void wait_for_execution() noexcept
    {
        m_started_sem.acquire();
    }

    void continue_execution() noexcept {
        m_block_sem.release();
    }

    void join() noexcept
    {
        m_thread.join();
    }

private:

    void thread_function() noexcept
    {
        m_started_sem.release();
        m_block_sem.acquire();
    }

    zpp::thread m_thread;
    zpp::binary_semaphore m_started_sem;
    zpp::binary_semaphore m_block_sem;

};

void test_thread_execution()
{
    thread_container thread(my_stack);
    thread.execute();
    const bool executed = thread.thread_executed();
    zassert_true(executed, "Thread not executed");
}

void test_thread_join()
{
    thread_container thread(my_stack);
    thread.execute();
    thread.wait_for_execution();
    thread.continue_execution();
    thread.join();
}

void test_main(void)
{
	ztest_test_suite(thread,
					 ztest_unit_test(test_thread_execution),
                     ztest_unit_test(test_thread_join));

	ztest_run_test_suite(thread);
}
