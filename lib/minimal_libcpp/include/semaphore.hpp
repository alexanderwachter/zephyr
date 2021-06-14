/*
 * Copyright (c) 2021 Leica Geosystems AG
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_LIB_MINIMAL_LIBCPP_MINIMAL_INCLUDE_SEMAPHORE_HPP
#define ZEPHYR_LIB_MINIMAL_LIBCPP_MINIMAL_INCLUDE_SEMAPHORE_HPP

#include <kernel.h>
#include <cstddef>
#include <limits>
#include <chrono>
#include "clock.hpp"

namespace zpp
{

// ptrdiff_t is int on ARM architecture -> unsigned int from the Zephyr API not possible
// using int to not overflow the Zephyr API on 64bit ABIs
template<std::ptrdiff_t LEAST_MAX_VALUE = std::numeric_limits<int>::max()>
class counting_semaphore
{
public:
    static constexpr std::ptrdiff_t max() noexcept {
        return LEAST_MAX_VALUE;
    }

    constexpr explicit counting_semaphore(std::ptrdiff_t desired = 0) noexcept
    {
        k_sem_init(&m_native_sem, static_cast<unsigned int>(desired), LEAST_MAX_VALUE);
    }

    ~counting_semaphore() noexcept = default;

    counting_semaphore(const counting_semaphore&) = delete;
    counting_semaphore& operator=(const counting_semaphore&) = delete;

    void release(std::ptrdiff_t update = 1) noexcept
    {
        for(; update != 0; --update) {
            k_sem_give(&m_native_sem);
        }
    }

    void acquire() noexcept
    {
        k_sem_take(&m_native_sem, K_FOREVER);
    }

    template<class Rep, class Period>
    bool try_acquire_for(std::chrono::duration<Rep, Period> const& rel_time) noexcept
    {

        return k_sem_take(&m_native_sem, chronoToTimeoutUsec(rel_time)) == 0;
    }

    bool try_acquire() noexcept
    {
        return k_sem_take(&m_native_sem, K_NO_WAIT) == 0;
    }

    template <class Clock, class Duration>
    bool try_acquire_until(std::chrono::time_point<Clock, Duration> const& abs_time) noexcept
    {
        auto const current = Clock::now();
        if(current >= abs_time)
            return try_acquire();
        else
            return try_acquire_for(abs_time - current);
    }

private:
    k_sem m_native_sem{};
};

using binary_semaphore = counting_semaphore<1>;

} // namespace zpp
#endif //ZEPHYR_LIB_MINIMAL_LIBCPP_MINIMAL_INCLUDE_SEMAPHORE_HPP
