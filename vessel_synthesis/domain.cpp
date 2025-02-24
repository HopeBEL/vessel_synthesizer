#include "domain.h"

#include <glm/gtc/constants.hpp>

#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtx/quaternion.hpp>

// DEBUG
#include <iostream>
#include <glm/gtx/string_cast.hpp>

namespace vs
{

/*void domain::dump_logs(std::string log_msg) {
    //if (!log_msg.empty()) m_logs = log_msg;
    m_logs = "Test";
}*/

void domain::samples(std::vector<glm::vec3> &points, std::size_t count)
{
    points.resize(count);
    for(auto& p : points)
    {
        p = sample();
    }
}

domain_circle::domain_circle(const glm::vec3 &center, float radius, const glm::vec3 &deadZonePos, float deadZoneRadius)
    : m_center(center), m_radius(radius), m_deadZonePos(deadZonePos), m_deadZoneRadius(deadZoneRadius),
      m_generator(42), m_distribution(0.0, 1.0), m_logs("Test")
{

}

void domain_circle::seed(unsigned int number)
{
    m_generator.seed(number);
}

glm::vec3 domain_circle::sample()
{
    float r = m_radius * std::sqrt(m_distribution(m_generator));
    float theta = m_distribution(m_generator) * 2.0f * glm::pi<float>();
    glm::vec3 pos = glm::vec3{ m_center.x + r * glm::cos(theta), m_center.y + r * glm::sin(theta), 0.0 };
    return pos;
    //if (glm::length(pos - m_deadZonePos) > m_deadZoneRadius) {
    //    std::cerr << "ici" << std::endl;
    //    printf("ici");
    //    fflush(stdout);
    //    FILE* log = fopen("C:/Users/aettaoui/Desktop/debug_log.txt", "a");
    //    fprintf(log, "ici\n");
    //    fclose(log);
//
    //    return pos;
    //}
    //else return { 0.0, 0.0, 0.0};
}

// Fills the vector repulsive_points (class attribute) with a new point
void domain_circle::add_repulsive_points(glm::vec3 point) {
    m_repulsive_points.push_back(point);
}

glm::vec3 domain_circle::min_extends() const
{
    return m_center - glm::vec3{m_radius, m_radius, m_radius};
}

glm::vec3 domain_circle::max_extends() const
{
    return m_center + glm::vec3{m_radius, m_radius, m_radius};
}

domain_sphere::domain_sphere(const glm::vec3 &center, float radius)
    : m_center(center), m_radius(radius),
      m_generator(42), m_uniform_distribution(-1.0, 1.0), m_normal_distribution(0.0, 1.0)
{

}

domain_halfsphere::domain_halfsphere(const glm::vec3 &center, float radius)
    : m_center(center), m_radius(radius),
      m_generator(42), m_uniform_distribution(-1.0, 1.0), m_normal_distribution(0.0, 1.0)
{

}

/*domain_lines_half_sphere::domain_lines_half_sphere(const glm::vec3 &center, float radius, const std::vector<glm::vec3>& start, const std::vector<glm::vec3>& end, float deviation)
    : m_center(center), m_radius(radius),
      m_generator(42), m_uniform_distribution(-1.0, 1.0), m_normal_distribution(0.0, 1.0),
      m_start(start), m_end(end), m_deviation(deviation),
      m_min(std::numeric_limits<float>::max()), m_max(-std::numeric_limits<float>::max())
{
     auto size = std::min(m_start.size(), m_end.size());
    for(auto i = 0u; i < size; i++)
    {
        m_min = glm::min( m_min, glm::min(m_start[i], m_end[i]) );
        m_max = glm::max( m_max, glm::max(m_start[i], m_end[i]) );
    }
}*/

void domain_sphere::seed(unsigned int number)
{
    m_generator.seed(number);
}

void domain_halfsphere::seed(unsigned int number)
{
    m_generator.seed(number);
}

/*void domain_lines_half_sphere::seed(unsigned int number)
{
    m_generator.seed(number);
}*/

glm::vec3 domain_sphere::sample()
{
    glm::vec3 pos =
    {
        m_normal_distribution(m_generator),
        m_normal_distribution(m_generator),
        m_normal_distribution(m_generator)
    };

    float d = std::pow( ((m_uniform_distribution(m_generator) + 1.0f) / 2.0f), 1.0f / 3.0f ) / glm::length(pos);
    return m_center + pos * d * m_radius;
}

glm::vec3 domain_halfsphere::sample()
{
    glm::vec3 pos =
    {
        m_normal_distribution(m_generator),
        m_normal_distribution(m_generator),
        std::abs(m_normal_distribution(m_generator))
    };

    float d = std::pow( ((m_uniform_distribution(m_generator) + 1.0f) / 2.0f), 1.0f / 3.0f ) / glm::length(pos);
    return m_center + pos * d * m_radius;
}

/*glm::vec3 domain_lines_half_sphere::sample()
{
    assert(!m_start.empty() && !m_end.empty());

    /* TODO: lines are currently selected independent on length! 
    auto size = std::min(m_start.size(), m_end.size());
    auto idx = static_cast<int>( std::round(m_distribution(m_generator) * (size-1)) );

    float r = m_deviation * std::sqrt(m_distribution(m_generator));
    float theta = m_distribution(m_generator) * 2.0f * glm::pi<float>();
    auto sample_circle = glm::vec3{ r * glm::cos(theta), r * glm::sin(theta), 0.0, };

    auto dir = m_end[idx] - m_start[idx];
    float length = glm::length(dir);
    dir = glm::normalize(dir);

    auto rotation = glm::mat3_cast(glm::rotation(glm::vec3{0.0, 0.0, 1.0}, dir));

    glm::vec3 point = (rotation * sample_circle) + length * m_distribution(m_generator) * dir + m_start[idx];


    glm::vec3 pos =
    {
        m_normal_distribution(m_generator),
        m_normal_distribution(m_generator),
        std::abs(m_normal_distribution(m_generator))
    };

    float d = std::pow( ((m_uniform_distribution(m_generator) + 1.0f) / 2.0f), 1.0f / 3.0f ) / glm::length(pos);
    return m_center + point + pos * d * m_radius;
}*/

glm::vec3 domain_sphere::min_extends() const
{
    return m_center - glm::vec3{m_radius, m_radius, m_radius};
}

glm::vec3 domain_sphere::max_extends() const
{
    return m_center + glm::vec3{m_radius, m_radius, m_radius};
}

glm::vec3 domain_halfsphere::min_extends() const
{
    return m_center - glm::vec3{m_radius, m_radius, m_radius};
}

glm::vec3 domain_halfsphere::max_extends() const
{
    return m_center + glm::vec3{m_radius, m_radius, m_radius};
}

/*glm::vec3 domain_lines_half_sphere::min_extends() const
{
    return m_center - glm::vec3{m_radius, m_radius, m_radius};
}

glm::vec3 domain_lines_half_sphere::max_extends() const
{
    return m_center + glm::vec3{m_radius, m_radius, m_radius};
}*/

domain_lines::domain_lines(const std::vector<glm::vec3> &start, const std::vector<glm::vec3> &end, float deviation)
    : m_start(start), m_end(end), m_deviation(deviation),
      m_min(std::numeric_limits<float>::max()), m_max(-std::numeric_limits<float>::max())
{
    auto size = std::min(m_start.size(), m_end.size());
    for(auto i = 0u; i < size; i++)
    {
        m_min = glm::min( m_min, glm::min(m_start[i], m_end[i]) );
        m_max = glm::max( m_max, glm::max(m_start[i], m_end[i]) );
    }
}

domain_lines::domain_lines(const std::vector<glm::vec3> &start, const std::vector<glm::vec3> &end, float sub_distance, float deviation)
    : m_deviation(deviation),
      m_min(std::numeric_limits<float>::max()), m_max(-std::numeric_limits<float>::max())
{
    auto size = std::min(start.size(), end.size());
    for(auto i = 0u; i < size; i++)
    {
        auto dir = end[i] - start[i];
        auto length = glm::length(dir);
        dir = glm::normalize(dir);

        int steps = std::max(int(length / sub_distance), 1);
        if(length - steps*sub_distance > 1e-8) { steps += 1; }

        for(int j = 0; j < steps; j++)
        {
            auto _start = start[i] + j * sub_distance * dir;
            auto _end = start[i] + (j+1) * sub_distance * dir;

            if(glm::length(start[i] - end[i]) > length)
            {
                _end = end[i];
            }

            m_start.emplace_back(_start);
            m_end.emplace_back(_end);
        }

        m_min = glm::min( m_min, glm::min(start[i], end[i]) );
        m_max = glm::max( m_max, glm::max(start[i], end[i]) );
    }
}

void domain_lines::seed(unsigned int number)
{
    m_generator.seed(number);
}

glm::vec3 domain_lines::sample()
{
    assert(!m_start.empty() && !m_end.empty());

    /* TODO: lines are currently selected independent on length! */
    auto size = std::min(m_start.size(), m_end.size());
    auto idx = static_cast<int>( std::round(m_distribution(m_generator) * (size-1)) );

    float r = m_deviation * std::sqrt(m_distribution(m_generator));
    float theta = m_distribution(m_generator) * 2.0f * glm::pi<float>();
    auto sample_circle = glm::vec3{ r * glm::cos(theta), r * glm::sin(theta), 0.0, };

    auto dir = m_end[idx] - m_start[idx];
    float length = glm::length(dir);
    dir = glm::normalize(dir);

    auto rotation = glm::mat3_cast(glm::rotation(glm::vec3{0.0, 0.0, 1.0}, dir));

    return (rotation * sample_circle) + length * m_distribution(m_generator) * dir + m_start[idx];
}

glm::vec3 domain_lines::min_extends() const
{
    return m_min - glm::vec3(m_deviation);
}

glm::vec3 domain_lines::max_extends() const
{
    return m_max + glm::vec3(m_deviation);
}

domain_voxels::domain_voxels(const glm::vec3& min, const glm::vec3& max, const glm::vec3& resolution, const std::vector<bool>& voxels)
    : m_min(min), m_max(max), m_voxel_size((max-min) / resolution),
      m_generator(42), m_distribution(0.0, 1.0)
{
    assert(voxels.size() >= resolution.x*resolution.y*resolution.z);

    for(int z = 0; z < resolution.z; z++)
    {
        for(int y = 0; y < resolution.y; y++)
        {
            for(int x = 0; x < resolution.x; x++)
            {
                if(voxels[z*resolution.y*resolution.x + y*resolution.x + x])
                {
                    glm::vec3 p = m_min + glm::vec3{(x + 0.5f)*m_voxel_size.x, (y + 0.5f)*m_voxel_size.y, (z + 0.5f)*m_voxel_size.z};
                    m_voxel_center.emplace_back(p);
                }
            }
        }
    }
}

domain_voxels::domain_voxels(const glm::vec3& min, const glm::vec3& max, const glm::vec3& resolution, const std::vector<glm::vec3>& voxels)
    : m_min(min), m_max(max), m_voxel_size((max-min) / resolution), m_voxel_center(voxels),
      m_generator(42), m_distribution(0.0, 1.0)
{

}

void domain_voxels::seed(unsigned int number)
{
    m_generator.seed(number);
}

glm::vec3 domain_voxels::sample()
{
    auto idx = static_cast<int>( std::round(m_distribution(m_generator) * (m_voxel_center.size()-1)) );
    const auto& sampled_voxel = m_voxel_center[idx];

    glm::vec3 offset =
    {
        m_distribution(m_generator) - 0.5f,
        m_distribution(m_generator) - 0.5f,
        m_distribution(m_generator) - 0.5f
    };
    offset = offset * m_voxel_size;

    return sampled_voxel + offset;
}

glm::vec3 domain_voxels::min_extends() const
{
    return m_min;
}

glm::vec3 domain_voxels::max_extends() const
{
    return m_max;
}

}
