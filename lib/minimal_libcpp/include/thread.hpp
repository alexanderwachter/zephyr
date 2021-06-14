/*
 * Copyright (c) 2021 Leica Geosystems AG
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_LIB_MINIMAL_LIBCPP_MINIMAL_INCLUDE_THREAD_HPP
#define ZEPHYR_LIB_MINIMAL_LIBCPP_MINIMAL_INCLUDE_THREAD_HPP

#include <Embedded/Eventing/Delegate.hpp>
#include <Mcu/clock.hpp>
#include <chrono>
#include <zephyr.h>

#define MCU_CREATE_STACK(symbol_, size_)                                               \
K_THREAD_STACK_DEFINE(symbol_##_stack, (size_));                                       \
static constexpr Mcu::ThreadStackC symbol_{.stack = symbol_##_stack, .size = (size_)};


namespace zpp
{

struct thread_stack
{
    k_thread_stack_t *stack;
    size_t size;
};

class thread
{
#ifdef CONFIG_FPU_SHARING
    static constexpr uint32_t safe_fpu_regs = K_FP_REGS;
#else
    static constexpr uint32_t safe_fpu_regs = 0;
#endif
public:
    using DelegateT = Embedded::Events::DelegateC<void(void)>;

    constexpr explicit thread(DelegateT delegate, const thread_stack stack, int prio, bool useFloat = false) noexcept
    : m_delegate(delegate)
    {
        k_thread_create(native_handle(), stack.stack, stack.size, thread_start,
                        this, nullptr, nullptr, prio, useFloat ? safe_fpu_regs : 0, K_FOREVER);
    }

    ~thread() noexcept
    {
        this->abort();
    }

    thread(const thread&) = delete;
    thread& operator=(const thread&) = delete;

    using native_handle_type = k_thread*;
    using id = k_thread*;
    inline native_handle_type native_handle() noexcept {return &m_native_thread;}

    inline void start() noexcept
    {
        k_thread_start(native_handle());
    }

    inline void abort() noexcept
    {
        k_thread_abort(native_handle());
    }

    inline bool joinable() noexcept
    {
        const auto ret = k_thread_join(native_handle(), K_NO_WAIT);
        return  ret == -EBUSY;
    }

    inline void join() noexcept
    {
        k_thread_join(native_handle(), K_FOREVER);
    }

    inline id get_id() noexcept {return native_handle(); }

private:

    static void thread_start(void *this_, void *, void*)
    {
        thread* this_thread = reinterpret_cast<thread*>(this_);
        this_thread->m_delegate();
    }

    k_thread m_native_thread{};
    DelegateT m_delegate{};
};

namespace this_thread
{
    static inline void yield() noexcept { k_yield(); }

    template<typename Rep, typename Period>
    static inline void sleep_for(const std::chrono::duration<Rep, Period>& sleep_duration) noexcept
    {
        k_sleep(chronoToTimeoutUsec(sleep_duration));
    }

    template <class Clock, class Duration>
    static inline void sleep_for(const std::chrono::time_point<Clock,Duration>& sleep_time) noexcept
    {
        const auto current = Clock::now();
        if(current >= sleep_time) {
            return;
        }
    
        return sleep_for(sleep_time - current);
    }
} // namespace this_thread


} // namespace zpp
#endif //ZEPHYR_LIB_MINIMAL_LIBCPP_MINIMAL_INCLUDE_THREAD_HPP
