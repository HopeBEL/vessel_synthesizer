#pragma once

#include <glm/glm.hpp>

#include <random>
#include <vector>

namespace vs
{
/*
 * ******************** [domain] ********************
 * - domains are solely defined by sampling points
 *      i.e. there is currently no enforcement of boundaries
 *
 *  - sample() creates a 3d point in space that is used by synthesizer to place attraction points
 *  - min_extends() and max_extends() are needed for the oc-tree
 *  - the result of the synthesizer is uniquely determined by the seed in the domain
 *      --> careful: std random are not standardized over platforms!
 */
struct domain
{
    public: std::string m_logs = "Test domain";
    virtual ~domain() = default;

    virtual void seed(unsigned int number) = 0;
    virtual glm::vec3 sample() = 0;
    virtual glm::vec3 sample_first_steps() = 0;
    virtual glm::vec3 min_extends() const = 0;
    virtual glm::vec3 max_extends() const = 0;
    //virtual void dump_logs(std::string log_msg);

    void samples(std::vector<glm::vec3>& points, std::size_t count);
    void samples_first_steps(std::vector<glm::vec3>& points, std::size_t count);
};


struct domain_circle : public domain
{
private:
    glm::vec3 m_center;
    float m_radius;
    glm::vec3 m_deadZonePos;
    std::mt19937 m_generator;
    std::uniform_real_distribution<float> m_distribution;

public:
    domain_circle(const glm::vec3& center = {0.0, 0.0, 0.0}, float radius = 1.0f, const glm::vec3& deadZonePos = {0.5, 0.0, 0.0}, float deadZoneRadius = 0.3f, int stepNoCrossing = 0, int stepBreakCrossingArtery = 0, int m_stepBreakCrossingVein = 0);
    ~domain_circle() = default;

    //std::string m_logs = "Test";
    float m_deadZoneRadius = 0.1f;
    int m_stepNoCrossing;
    int m_stepBreakCrossingArtery;
    int m_stepBreakCrossingVein;
    std::vector<glm::vec3> m_crossingPointsBifurc;
    std::vector<glm::vec3> m_crossingPointsBifurcNew;
    std::vector<glm::vec3> m_crossingPointsSingle;
    std::vector<glm::vec3> m_crossingPointsSingleNew;

    std::vector<glm::vec3> m_repulsive_points;
    void seed(unsigned int number = 42) override;
    glm::vec3 sample() override;
    glm::vec3 sample_first_steps() override;
    virtual glm::vec3 min_extends() const override;
    virtual glm::vec3 max_extends() const override;
    //virtual void dump_logs(std::string log_msg) override;
    void add_repulsive_points(glm::vec3 repulsive_points);
};


struct domain_sphere : public domain
{
private:
    glm::vec3 m_center;
    float m_radius;

    std::mt19937 m_generator;
    std::uniform_real_distribution<float> m_uniform_distribution;
    std::normal_distribution<float> m_normal_distribution;

public:
    domain_sphere(const glm::vec3& center = {0.0, 0.0, 0.0}, float radius = 1.0f);
    ~domain_sphere() = default;
    
    //std::string m_logs;
    void seed(unsigned int number = 42) override;
    glm::vec3 sample() override;
    glm::vec3 sample_first_steps() override;
    virtual glm::vec3 min_extends() const override;
    virtual glm::vec3 max_extends() const override;
    
    //virtual void dump_logs(std::string log_msg) override;
};

struct domain_halfsphere : public domain
{
private:
    glm::vec3 m_center;
    float m_radius;

    std::mt19937 m_generator;
    std::uniform_real_distribution<float> m_uniform_distribution;
    std::normal_distribution<float> m_normal_distribution;

public:
    domain_halfsphere(const glm::vec3& center = {0.0, 0.0, 0.0}, float radius = 1.0f);
    ~domain_halfsphere() = default;

    void seed(unsigned int number = 42) override;
    glm::vec3 sample() override;
    glm::vec3 sample_first_steps() override;
    virtual glm::vec3 min_extends() const override;
    virtual glm::vec3 max_extends() const override;
};


/*
 * ******************** [line domain] ********************
 * -> this can be used to develop an initial tree; not really ideal/working (more of a proof-of-concept)
 */
struct domain_lines : public domain
{
private:
    std::vector<glm::vec3> m_start;
    std::vector<glm::vec3> m_end;

    float m_deviation;
    glm::vec3 m_min;
    glm::vec3 m_max;

    std::mt19937 m_generator;
    std::uniform_real_distribution<float> m_distribution;

public:
    domain_lines(const std::vector<glm::vec3>& start, const std::vector<glm::vec3>& end, float deviation);
    domain_lines(const std::vector<glm::vec3>& start, const std::vector<glm::vec3>& end, float sub_distance, float deviation);
    ~domain_lines() = default;

    void seed(unsigned int number = 42) override;
    glm::vec3 sample() override;
    glm::vec3 sample_first_steps() override;
    virtual glm::vec3 min_extends() const override;
    virtual glm::vec3 max_extends() const override;
};

/*
 * ******************** [voxel domain] ********************
 * -> either by boolean array indicating voxel locations (true)
 * -> or directly feedings voxel centers
 */
struct domain_voxels : public domain
{
private:
    glm::vec3 m_min;
    glm::vec3 m_max;

    glm::vec3 m_voxel_size;
    std::vector<glm::vec3> m_voxel_center;

    std::mt19937 m_generator;
    std::uniform_real_distribution<float> m_distribution;

public:
    domain_voxels(const glm::vec3& min, const glm::vec3& max, const glm::vec3& resolution, const std::vector<bool>& voxels);
    domain_voxels(const glm::vec3& min, const glm::vec3& max, const glm::vec3& resolution, const std::vector<glm::vec3>& voxels);
    ~domain_voxels() = default;

    void seed(unsigned int number = 42) override;
    glm::vec3 sample() override;
    glm::vec3 sample_first_steps() override;
    virtual glm::vec3 min_extends() const override;
    virtual glm::vec3 max_extends() const override;

};

}
