/*
 * Copyright (c) 2021 Leica Geosystems AG
 */

/**
 * The cxa_guard is part of the C++ ABI 3.2.3 (https://developer.arm.com/documentation/ihi0041/latest)
 * The guard ensures that static local objects/variables are initialized only once (ctor called only once).
 */

#include <app_memory/app_memdomain.h>
#include <sys/mutex.h>
#include <exception>

#if defined(CONFIG_ARM)
using guard_type = uint32_t;
#else
using guard_type = uint64_t;
#endif

static K_APP_DMEM(z_libc_partition) SYS_MUTEX_DEFINE(__cxa_guard_mtx);

class cxa_guard_impl {
public:
    explicit cxa_guard_impl(guard_type* guard_object) noexcept
        : m_guard_byte(reinterpret_cast<uint8_t*>(guard_object)) {}

    int acquire() const noexcept
    {
        if (*m_guard_byte == COMPLETE) {
            return INIT_IS_DONE;
        }

        lock();

        if (*m_guard_byte == COMPLETE) {
            return INIT_IS_DONE;
        }

        return INIT_IS_PENDING;
    }

    void release() const noexcept
    {
        *m_guard_byte = COMPLETE;
        unlock();
    }

    void abort() const noexcept
    {
        unlock();
    }

private:
    void lock() const noexcept
    {
        int res = sys_mutex_lock(guard_mutex, K_FOREVER);
        if (res != 0) {
            std::terminate();
        }
    }

    void unlock() const noexcept
    {
        int res = sys_mutex_unlock(guard_mutex);
        if (res != 0) {
            std::terminate();
        }
    }

    static constexpr uint8_t UNINIT = 0;
    static constexpr uint8_t COMPLETE = (1 << 0);
    static constexpr int INIT_IS_DONE = 0;
    static constexpr int INIT_IS_PENDING = 1;
    static constexpr sys_mutex* guard_mutex = &__cxa_guard_mtx;
    uint8_t *m_guard_byte;
};

extern "C"
{

int __cxa_guard_acquire(guard_type* raw_guard_object)
{
    cxa_guard_impl impl(raw_guard_object);
    return impl.acquire();
}

void __cxa_guard_release(guard_type *raw_guard_object) {
    cxa_guard_impl impl(raw_guard_object);
    impl.release();
}

void __cxa_guard_abort(guard_type *raw_guard_object) {
    cxa_guard_impl impl(raw_guard_object);
    impl.abort();
}

}  // extern "C"
