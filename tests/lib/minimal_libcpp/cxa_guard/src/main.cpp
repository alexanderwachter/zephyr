#include <ztest.h>
#include <kernel.h>

K_THREAD_STACK_DEFINE(stack, 256);

static struct k_thread thread;

static constexpr int initValue = 42;

struct nontrivial_class
{
    nontrivial_class() noexcept: m_val()
    {
        // trigger a context switch while holding the lock
        k_yield();
        zassert_equal(m_val, 0, "static memory (BSS segment) must be 0");
        m_val = initValue;
    }

    int m_val;
};

void function_with_static_object(int offset)
{
    static nontrivial_class object;
    zassert_equal(object.m_val, initValue + offset, "ctor not called");
    object.m_val++;
}

void test_init_guard(void)
{
    function_with_static_object(0);
    function_with_static_object(1);
}

void multithread_function_with_static_object(void)
{
    static nontrivial_class object;
    zassert_equal(object.m_val, initValue, "ctor not called");
}

void thread_function(void*, void*, void*)
{
    multithread_function_with_static_object();
}

void test_init_guard_multithread(void)
{
    k_thread_create(&thread, stack, K_THREAD_STACK_SIZEOF(stack), thread_function,
		NULL, NULL, NULL, CONFIG_MAIN_THREAD_PRIORITY - 1,  0, K_NO_WAIT);
    
    multithread_function_with_static_object();
    
    k_thread_join(&thread, K_FOREVER);
}

void test_main(void)
{
	ztest_test_suite(cxa_guard,
                     ztest_unit_test(test_init_guard),
                     ztest_unit_test(test_init_guard_multithread));

	ztest_run_test_suite(cxa_guard);
}
