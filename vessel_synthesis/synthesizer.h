#pragma once

#include "binarytree.h"
#include "forest.h"
#include "domain.h"
#include "octree.h"
#include "points.h"
#include "profiler.h"

#include <atomic>

namespace vs
{

enum class system : int { arterial = 0, venous = 1, count = 2 };
enum class grow_func : int { none = 0, linear = 1, exponential = 2, count = 2 };

/*
 * ******************** [synthesizer settings] ********************
 * - for both the arterial and venous system
 *
 * - steps: number of iterations (arterial and venous development step)
 * - sample_count: the number of attraction points sampled in each step
 *
 * - for both arterial and venous system:
 *      - parent_inertia: enforces to follow the direction of the parent ("stiffness" of tree)
 *      - birth_attr: distance between attraction points
 *      - birth_node: distance of attraction points to vessel nodes
 *      - influence attr: influence distance to attr points where vessel nodes start to grow
 *      - kill attr: distance to consider an attr point as satisfied
 *      - percept_vol: perception volume (search cone to finde attr points)
 *      - term_radius: terminal radius of vessel tree
 *      - growth_distance: distance that leaf can grow per step
 *      - bif_thresh: standard deviation threshold that initiates a bifurcation development
 *      - bif_index: exponent of murrays law
 *
 *      - grow func: type and value to change distances over time
 *          -> this has a significant impact on size of tree compared to the domain
 *          -> in combination with sample_count this has significant impact on the performance
 *
 *
 * -> for the venous system it makes sense to increase the influence distance
 * and decrease the birth_node distance a bit compared to the arterial system;
 * otherwise it may not be able to follow the growth of the arterial trees
 *
*/
struct settings
{
    unsigned int m_steps{100};
    unsigned int m_sample_count{1000};

    struct system
    {
        float m_parent_inertia{0.5f};

        float m_birth_attr{0.03f};
        float m_birth_node{0.04f};

        float m_influence_attr{0.065f};
        float m_kill_attr{0.8f};
        float m_percept_vol{90.0f};

        float m_term_radius{0.0005f};
        float m_growth_distance{0.02f};

        float m_bif_thresh{15.0f};
        float m_bif_index{2.1f};

        struct
        {
            grow_func m_type{grow_func::linear};
            float m_value{0.02};
        } m_grow_func;

        /* constraints */
        bool m_only_leaf_development{false};
    } m_system[static_cast<int>(vs::system::count)];

public:
    void scale(float s);

    settings::system& get_system_data(const vs::system sys);
};



/*
 * ******************** [synthesizer] ********************
 * - vessel synthesizer based on constraint space filling
 * i.e. it develops iteratively a tree while enforcing "soft" constraints on the geometry (radius, angle) resembling vascular patterns
 *
 * 1. "growth domain" needs to be defined --> see domain.h
 * 2. arterial tree roots or an initial vascular trees can be set from which the development starts (create_root(), set_forest())
 * 3. adjust settings (get_settings())
 * 4. run()
 * 5. retrieve developed trees for each system (get_forest())
 *
 * dev notes:
 * -> this version is single threaded, and uses an oc-tree for nearest neighbour searches
 * -> it is a bit messy at times
 *         -> most notably there is a raw ptr in node, to access its owning tree)
 *         -> remove attr points by look up the location
 *         -> oc-tree doesnt balance or collapses any nodes
*/
struct synthesizer
{
    using tree = vs::binary_tree<node_data>;
    using forest = vs::forest<node_data>;
    using attr = vs::attr_point<>;

    using oc_tree_attr = util::oc_tree<glm::vec3, 3, attr>;
    using oc_tree_node = util::oc_tree<glm::vec3, 3, tree::node*>;


    /* scaled distance parameters over time */
    struct parameter
    {
        unsigned int m_curr_step{0};

        struct system
        {
            float m_birth_attr{0.03f};
            float m_birth_node{0.04f};

            float m_influence_attr{0.065f};
            float m_kill_attr{0.03f};
            float m_growth_distance{0.03f};

            float repulsive_threshold{0.25f};
            float m_scaling{1.0};
        } m_system[2];
    };

    /* arterial and venous system data */
    struct system_data
    {
        forest m_forest;
        oc_tree_attr m_attr_search;
        oc_tree_node m_node_search;
        std::vector<glm::vec3> m_killed_attr;
        prf::monitor m_profiler;

    public:
        system_data(const glm::vec3& min, const glm::vec3& max);
        void clear();
        void clear_attr();
    };


private:
    std::reference_wrapper<domain> m_domain;
    settings m_settings;
    parameter m_params;
    system_data m_systems[static_cast<int>(system::count)];

    std::atomic_bool m_is_running;


public:
    synthesizer(domain& tissue);

    void set_settings(const settings& sett);
    settings& get_settings();
    settings::system& get_system_settings(const system sys);

    parameter& get_parameter();
    parameter::system& get_system_parameter(const system sys);

    system_data& get_system_data(const system sys);

    const forest& get_forest(const system sys);
    void set_forest(const system sys, const forest& other);

    tree::node& create_root(const system sys, const glm::vec3& pos);
    void create_attr(const system sys, const glm::vec3& pos);
    void try_attr(const system sys, const glm::vec3& pos, int& i);

    void run();

private:
    void init_runtime_params();

    void step(const system sys);
    void sample_attraction();
    void step_closest(const system sys, std::map< tree::node*, std::list<attr> >& attr_map);
    void step_growth(const system sys, std::map< tree::node*, std::list<attr> >& attr_map);
    void step_kill(const system sys, std::map< tree::node*, std::list<attr> >& attr_map);
    void combine_systems();

    void domain_growth(const system sys);
};

}
