/*
 * Copyright (c) 2021 Leica Geosystems AG
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_LIB_MINIMAL_LIBCPP_MINIMAL_INCLUDE_CONDITION_VARIABLE_HPP
#define ZEPHYR_LIB_MINIMAL_LIBCPP_MINIMAL_INCLUDE_CONDITION_VARIABLE_HPP

#include "mutex.hpp"
#include <kernel.h>
#include <chrono>
// for std::unique_lock
#include <mutex>

namespace zpp
{

enum class cv_status { no_timeout, timeout };

class condition_variable {
public:
    constexpr condition_variable() noexcept { k_condvar_init(&m_native_cond_var); }

    condition_variable(const condition_variable&) = delete;
    condition_variable& operator=(const condition_variable&) = delete;

    ~condition_variable() noexcept = default;

    inline void notify_one() noexcept
    {
        k_condvar_signal(native_handle());
    }

    inline void notify_all() noexcept
    {
        k_condvar_broadcast(native_handle());
    }

    inline void wait(std::unique_lock<recursive_mutex>& lock)
    {
        const auto mtx = lock.mutex();
        k_condvar_wait(native_handle(), mtx->native_handle(), K_FOREVER);
    }

    template <class Predicate>
    void wait(std::unique_lock<recursive_mutex>& lock, Predicate pred)
    {
        while (!pred()) {
            wait(lock);
        }
    }

    template <class Rep, class Period>
    cv_status wait_for(std::unique_lock<recursive_mutex>& lock, const std::chrono::duration<Rep, Period>& rel_time) noexcept
    {
        const auto mtx = lock.mutex();
        const auto ret = k_condvar_wait(native_handle(), mtx->native_handle(), chronoToTimeoutUsec(rel_time));
        return ret == 0 ? cv_status::no_timeout : cv_status::timeout;
    }

    template <class Rep, class Period, class Predicate>
    bool wait_for(std::unique_lock<recursive_mutex>& lock, const std::chrono::duration<Rep, Period>& rel_time, Predicate pred)
    {
        while (!pred()) {
            if (wait_for(lock, rel_time) == cv_status::timeout)
                return pred();
        }
    }

    template <class Clock, class Duration>
    cv_status wait_until(std::unique_lock<recursive_mutex>& lock, const std::chrono::time_point<Clock, Duration>& abs_time)
    {
        auto const current = Clock::now();
        if(current >= abs_time)
            return cv_status::timeout;
        else
            return wait_for(lock, abs_time - current);
    }

    template <class Clock, class Duration, class Predicate>
    bool wait_until(std::unique_lock<recursive_mutex>& lock, const std::chrono::time_point<Clock, Duration>& abs_time, Predicate pred)
    {
        while (!pred()) {
            if (wait_until(lock, abs_time) == cv_status::timeout)
                return pred();
        }
    }

    using native_handle_type = k_condvar*;
    inline native_handle_type native_handle() noexcept {return &m_native_cond_var;}

private:
    k_condvar m_native_cond_var{};
};

} // namespace zpp
#endif //ZEPHYR_LIB_MINIMAL_LIBCPP_MINIMAL_INCLUDE_CONDITION_VARIABLE_HPP
