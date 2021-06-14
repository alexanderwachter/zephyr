/*
 * Copyright (c) 2021 Leica Geosystems AG
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_LIB_MINIMAL_LIBCPP_MINIMAL_INCLUDE_MUTEX_HPP
#define ZEPHYR_LIB_MINIMAL_LIBCPP_MINIMAL_INCLUDE_MUTEX_HPP

#include <kernel.h>
#include <chrono>
#include "clock.hpp"

namespace Mcu
{

class recursive_mutex {
public:
    constexpr recursive_mutex() noexcept { k_mutex_init(native_handle()); }

    recursive_mutex(const recursive_mutex&) = delete;
    recursive_mutex& operator=(const recursive_mutex&) = delete;

    ~recursive_mutex() noexcept = default;

    inline void lock() noexcept { k_mutex_lock(native_handle(), K_FOREVER); }
    inline bool try_lock() noexcept { return k_mutex_lock(native_handle(), K_NO_WAIT) == 0; }
    inline void unlock() noexcept { k_mutex_unlock(native_handle()); }

    using native_handle_type = k_mutex*;
    inline native_handle_type native_handle() noexcept {return &m_native_mutex;}

private:
    k_mutex m_native_mutex{};
};

class recursive_timed_mutex {
public:
    constexpr recursive_timed_mutex() noexcept { k_mutex_init(native_handle()); }

    recursive_timed_mutex(const recursive_timed_mutex&) = delete;
    recursive_timed_mutex& operator=(const recursive_timed_mutex&) = delete;

    ~recursive_timed_mutex() noexcept = default;

    inline void lock() noexcept { k_mutex_lock(native_handle(), K_FOREVER); }
    inline bool try_lock() noexcept { return k_mutex_lock(native_handle(), K_NO_WAIT) == 0; }
    inline void unlock() noexcept { k_mutex_unlock(native_handle()); }
    template <class Rep, class Period>
    bool try_lock_for(const std::chrono::duration<Rep, Period>& rel_time) noexcept
    {
        return k_mutex_lock(native_handle(), chronoToTimeoutUsec(rel_time)) == 0;;
    }

    template <class Clock, class Duration>
    bool try_lock_until(const std::chrono::time_point<Clock, Duration>& abs_time) noexcept
    {
        auto const current = Clock::now();
        if(current >= abs_time)
            return try_lock();
        else
            return try_lock_for(abs_time - current);
    }

    using native_handle_type = k_mutex*;
    inline native_handle_type native_handle() noexcept {return &m_native_mutex;}

private:
    k_mutex m_native_mutex{};
};

} // namespace zpp
#endif //ZEPHYR_LIB_MINIMAL_LIBCPP_MINIMAL_INCLUDE_MUTEX_HPP
