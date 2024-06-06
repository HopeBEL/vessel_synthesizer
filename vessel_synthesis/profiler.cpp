#include "profiler.h"

namespace vs::prf
{

template<>
double time_cast<nano_seconds>(const duration& t)
{
    return std::chrono::duration_cast<nano_seconds>(t).count();
}

template<>
double time_cast<micro_seconds>(const duration& t)
{
    return std::chrono::duration_cast<micro_seconds>(t).count();
}

template<>
double time_cast<milli_seconds>(const duration& t)
{
    return std::chrono::duration<double, std::ratio<1, 1000>>(t).count();
}

cpu_sample::cpu_sample(const char *name, monitor& profiler)
    : m_name(name), m_profiler(profiler), m_start(highres_clock::now())
{

}

cpu_sample::~cpu_sample()
{
    auto end = highres_clock::now();
   m_profiler.add_time(m_name, end - m_start);
}

void monitor::start_frame()
{

}

void monitor::end_frame()
{
    m_frame_count++;
}

void monitor::add_time(const std::string &name, duration t)
{
    auto& samples = m_samples[name];
    if(samples.size() <= static_cast<std::size_t>(m_frame_count))
    {
        samples.resize(m_frame_count + 1, duration::zero());
    }

    samples.back() += t;
}

void monitor::reset()
{
    m_frame_count = 0;
    m_samples.clear();
}

const monitor::profile_samples &monitor::get_samples()
{
    for(auto& kv : m_samples)
    {
        if(kv.second.size() < static_cast<std::size_t>(m_frame_count))
        {
            kv.second.resize(m_frame_count, duration::zero());
        }
    }

    return m_samples;
}

}
