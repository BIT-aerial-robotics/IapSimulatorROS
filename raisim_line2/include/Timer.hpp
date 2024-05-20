// Timer.hpp
// functions to control loop time
// migrite from Windows
// Du Jianrui

#include <chrono>

inline long long getSystemTimeNanos(std::chrono::system_clock::time_point* t = nullptr)
{
    //high res clock has epoch since boot instead of since 1970 for VC++
    return std::chrono::duration_cast<std::chrono::nanoseconds>((t != nullptr ? *t : std::chrono::system_clock::now())
        .time_since_epoch())
        .count();
}

inline long long getTimeElapsedNanos(long long& lastTimeNanos_)
{
    auto currentTimeNanos = getSystemTimeNanos();
    auto intervalNanos = currentTimeNanos - lastTimeNanos_;
    lastTimeNanos_ = currentTimeNanos;

    return intervalNanos;
}

inline void loopSleepForCertainTime(long long lastTimeNanos_, long long millisecond_)
{
    long long timePeriodNanos = millisecond_ * 1000000;
    while (timePeriodNanos > getSystemTimeNanos() - lastTimeNanos_)
    {
        // does nothing
        // just loops to kill time
    }
}
