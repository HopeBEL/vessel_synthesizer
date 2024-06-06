#pragma once

#include <chrono>
#include <vector>
#include <map>
#include <string>

namespace vs::prf
{

typedef std::conditional<std::chrono::high_resolution_clock::is_steady, std::chrono::high_resolution_clock, std::chrono::steady_clock >::type  highres_clock;
typedef highres_clock::time_point    time_point;
typedef highres_clock::duration      duration;

typedef std::chrono::nanoseconds    nano_seconds;
typedef std::chrono::microseconds   micro_seconds;
typedef std::chrono::milliseconds   milli_seconds;

template <typename T> double time_cast(const duration& time) { return std::chrono::duration_cast<T>(time).count(); }
template<> double time_cast<nano_seconds>(const duration& t);
template<> double time_cast<micro_seconds>(const duration& t);
template<> double time_cast<milli_seconds>(const duration& t);

/*
 * ******************** [profile monitor] ********************
 * -> simple performance monitoring; collecting per frame time measurements
 *
 */
struct monitor
{
    using frame_times = std::vector<duration>;
    using profile_samples = std::map<std::string, frame_times>;

#ifdef VS_PROFILER
    static constexpr bool is_enabled = true;
#else
    static constexpr bool is_enabled = false;
#endif

private:
    profile_samples m_samples;
    int m_frame_count{0};

public:
    void start_frame();
    void end_frame();

    void add_time(const std::string& name, duration t);
    void reset();

    const profile_samples& get_samples();
};


struct cpu_sample final
{
    cpu_sample(const char* name, monitor& profiler);
    ~cpu_sample();

private:
    const char* m_name;
    monitor& m_profiler;
    time_point m_start;
};

#ifdef VS_PROFILER
#define profile_sample(name, profiler) vs::prf::cpu_sample sample_ ## name(#name, profiler)
#else
#define profile_sample(name, profiler)
#endif

}
