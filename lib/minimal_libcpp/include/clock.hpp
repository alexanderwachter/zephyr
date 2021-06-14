/*
 * Copyright (c) 2021 Leica Geosystems AG
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_LIB_MINIMAL_LIBCPP_MINIMAL_INCLUDE_CLOCK_HPP
#define ZEPHYR_LIB_MINIMAL_LIBCPP_MINIMAL_INCLUDE_CLOCK_HPP

#include <chrono>
#include <cstdint>
#include <sys/time_units.h>
#include <sys_clock.h>
#include <kernel.h>

namespace zpp
{
namespace chrono
{
struct steady_clock
{
    using rep        = std::int64_t;
    using period     = std::ratio<1, k_ms_to_ticks_floor64(MSEC_PER_SEC)>;
    using duration   = std::chrono::duration<rep, period>;
    using time_point = std::chrono::time_point<steady_clock>;
    static constexpr bool is_steady = true;

    static time_point now() noexcept
    {
        const auto now = k_uptime_ticks();
        return time_point{duration{now}};
    }
};

} //namespace chrono

template<class Rep, class Period>
static inline auto chrono_to_timeout_usec(std::chrono::duration<Rep, Period> const& time) {
#ifdef CONFIG_TIMEOUT_64BIT
    using ValueType = std::uint64_t;
#else
    using ValueType = std::uint32_t;
#endif
    return K_USEC(static_cast<ValueType>(std::chrono::microseconds(time).count()));
}

} // namespace zpp
#endif //ZEPHYR_LIB_MINIMAL_LIBCPP_MINIMAL_INCLUDE_CLOCK_HPP
