#include <pybind11/pybind11.h>
#include <pybind11/chrono.h>
#include <pybind11/functional.h>
#include <pybind11/iostream.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include <vessel_synthesis/domain.h>
#include <vessel_synthesis/synthesizer.h>

#include "glm_cast.h"

namespace py = pybind11;

/*********** [Python Bindings] ***************
 * pybind11 bindings for vessel synthesizer
 *
 * -> cmake will generate a .pyd file in build directory "lib"
 *    currently i do not statically link the c++ libraries; you'll need to move the necessary libs to the folder
 *    e.g. for mingw on windows i need to add libgcc_s_seh-1.dll, libstdc++-6.dll, libwinpthread-1.dll
 *
 * -> in python you need to add the path to this lib directory in order for import to find it
 *      import sys
 *      sys.path.insert(0, '../build/MinGW_Desktop-Release/lib/')
 *      import vessel_module as vs
 *
 * Some developer notes:
 *  -> I dont have any iostream output, but for debugging it can be convinient to redirect it to python:
 *     last argument to .def() --> function: py::call_guard<py::scoped_ostream_redirect, py::scoped_estream_redirect>()
 *     e.g. .def("min_extends", &vs::domain::min_extends, py::call_guard<py::scoped_ostream_redirect, py::scoped_estream_redirect>())
 *
 *  -> automatic python "to string" could be supported with .def("__repr__", ...); not implemented right now
 *  -> for documentation you can use .doc() = ""; I haven't done this yet
 *
 *  -> do not edit the topology of the tree while iterating with (breadth_first, depth_first, ...)!
 *
 *  -> I did not spend a lot of time optimizing data access (lot of data copy for conversion to python container)
 *  -> this is the single threaded synthesizer which should be fast enough for prototyping
 */

PYBIND11_MODULE(vessel_module, m)
{
    m.doc() = "Vessel-Synthesizer Module";

    /****************************************************
     *                   Domain Geometry                *
     ****************************************************/
    py::class_<vs::domain>(m, "Domain")
            .def("seed", &vs::domain::seed)
            .def("min_extends", &vs::domain::min_extends)
            .def("max_extends", &vs::domain::max_extends)
            .def("sample", &vs::domain::sample)
            .def("samples", [](vs::domain& self, unsigned int count)
            {
                std::vector<glm::vec3> _samples;
                self.samples(_samples, count);
                return _samples;
            })
            .def("dump_logs", [](vs::domain& self) {
                std::string test;

                if (!self.m_logs.empty())
                    test = self.m_logs;
                else test = "empty";
                return test;
            });

    py::class_<vs::domain_circle, vs::domain>(m, "DomainCircle")
            .def(py::init<const glm::vec3&, float, const glm::vec3&, float, int, int, int >())
            .def("seed", &vs::domain_circle::seed)
            .def("min_extends", &vs::domain_circle::min_extends)
            .def("max_extends", &vs::domain_circle::max_extends)
            .def("sample", &vs::domain_circle::sample)
            .def("samples", [](vs::domain& self, unsigned int count)
            {
                std::vector<glm::vec3> _samples;
                self.samples(_samples, count);
                return _samples;
            })
            .def("add_repulsive_points", &vs::domain_circle::add_repulsive_points)
            .def("get_crossing_points_bifurc", [](vs::domain_circle& self) {
                return self.m_crossingPointsBifurc;
            })
            .def("get_crossing_points_single", [](vs::domain_circle& self) {
                return self.m_crossingPointsSingle;
            })
            .def("get_crossing_points_bifurc_new", [](vs::domain_circle& self) {
                return self.m_crossingPointsBifurcNew;
            })
            .def("get_crossing_points_single_new", [](vs::domain_circle& self) {
                return self.m_crossingPointsSingleNew;
            })


            ;


    py::class_<vs::domain_sphere, vs::domain>(m, "DomainSphere")
            .def(py::init<const glm::vec3&, float>())
            .def("seed", &vs::domain_sphere::seed)
            .def("min_extends", &vs::domain_sphere::min_extends)
            .def("max_extends", &vs::domain_sphere::max_extends)
            .def("sample", &vs::domain_sphere::sample)
            .def("samples", [](vs::domain& self, unsigned int count)
            {
                std::vector<glm::vec3> _samples;
                self.samples(_samples, count);
                return _samples;
            });

    py::class_<vs::domain_halfsphere, vs::domain>(m, "DomainHalfSphere")
            .def(py::init<const glm::vec3&, float>())
            .def("seed", &vs::domain_halfsphere::seed)
            .def("min_extends", &vs::domain_halfsphere::min_extends)
            .def("max_extends", &vs::domain_halfsphere::max_extends)
            .def("sample", &vs::domain_halfsphere::sample)
            .def("samples", [](vs::domain& self, unsigned int count){
                std::vector<glm::vec3> _samples;
                self.samples(_samples, count);
                return _samples;
            });

    py::class_<vs::domain_lines, vs::domain>(m, "DomainLines")
            .def(py::init<const std::vector<glm::vec3>&, const std::vector<glm::vec3>&, float>())
            .def(py::init<const std::vector<glm::vec3>&, const std::vector<glm::vec3>&, float, float>())
            .def("seed", &vs::domain_lines::seed)
            .def("min_extends", &vs::domain_lines::min_extends)
            .def("max_extends", &vs::domain_lines::max_extends)
            .def("sample", &vs::domain_lines::sample)
            .def("samples", [](vs::domain& self, unsigned int count)
            {
                std::vector<glm::vec3> _samples;
                self.samples(_samples, count);
                return _samples;
            });

    py::class_<vs::domain_voxels, vs::domain>(m, "DomainVoxels")
            .def(py::init<const glm::vec3&, const glm::vec3&, const glm::vec3&, const std::vector<bool>&>())
            .def(py::init<const glm::vec3&, const glm::vec3&, const glm::vec3&, const std::vector<glm::vec3>&>())
            .def("seed", &vs::domain_voxels::seed)
            .def("min_extends", &vs::domain_voxels::min_extends)
            .def("max_extends", &vs::domain_voxels::max_extends)
            .def("sample", &vs::domain_voxels::sample)
            .def("samples", [](vs::domain& self, unsigned int count)
            {
                std::vector<glm::vec3> _samples;
                self.samples(_samples, count);
                return _samples;
            });



    /****************************************************
     *                   Vessel Node                    *
     ****************************************************/
    using vs_node = vs::synthesizer::tree::node;
    py::class_<vs_node>(m, "Node")
            .def_property_readonly("id", &vs_node::id)
            .def_property_readonly("parent", &vs_node::parent)
            .def_property_readonly("children", &vs_node::children)
            .def_property("position",
                    [](const vs_node& self) { return self.data().m_pos; },
                    [](vs_node& self, const glm::vec3& p) { self.data().m_pos = p; })
            .def_property("radius",
                    [](const vs_node& self) { return self.data().m_radius; },
                    [](vs_node& self, float r) { self.data().m_radius = r; })
            .def("is_root", &vs_node::is_root)
            .def("is_inter", &vs_node::is_inter)
            .def("is_leaf", &vs_node::is_leaf)
            .def("is_joint", &vs_node::is_joint);
    
    /****************************************************
     *                    Vessel Tree                   *
     ****************************************************/
    using vs_tree = vs::synthesizer::tree;
    py::class_<vs_tree>(m, "Tree")
            .def(py::init<>())
            .def("create_root", &vs_tree::create_root<>, py::return_value_policy::reference)
            .def("create_node", &vs_tree::create_node<>, py::return_value_policy::reference)
            .def("delete_node", &vs_tree::delete_node)
            .def("get_root", static_cast<vs_node& (vs_tree::*)()>(&vs_tree::get_root), py::return_value_policy::reference)
            .def("get_node", static_cast<vs_node& (vs_tree::*)(vs::node_id)>(&vs_tree::get_node), py::return_value_policy::reference)
            .def("node_map", static_cast<const std::unordered_map<vs::node_id, vs_node>& (vs_tree::*)() const>(&vs_tree::get_all_nodes))
            .def("size", &vs_tree::size)
            .def("breadth_first", [](vs_tree& self, const std::function<void(vs_node&)>& func) { self.breadth_first(func); })
            .def("depth_first", [](vs_tree& self, const std::function<void(vs_node&)>& func) { self.depth_first(func); })
            .def("post_order", [](vs_tree& self, const std::function<void(vs_node&)>& func) { self.post_order(func); })
            .def("delete_if", [](vs_tree& self, const std::function<bool(vs_node&)>& func) { self.delete_if(func); })
            .def("segment_data", [](vs_tree& self)
            {
                std::vector<glm::vec3> start;
                std::vector<glm::vec3> end;
                std::vector<float> radius;

                start.reserve(2*self.size());
                end.reserve(2*self.size());
                radius.reserve(2*self.size());

                self.breadth_first([&start, &end, &radius, &self](auto& n)
                {
                    if(!n.is_root())
                    {
                        start.emplace_back(self.get_node(n.parent()).data().m_pos);
                        end.emplace_back(n.data().m_pos);
                        radius.emplace_back(n.data().m_radius);
                    }
                });

                return std::make_tuple(start, end, radius);
            });

    /****************************************************
     *                      Forest                      *
     ****************************************************/
    using vs_forest = vs::synthesizer::forest;
    py::class_<vs_forest>(m, "Forest")
            .def("trees", static_cast<std::list<vs_tree>& (vs_forest::*)()>(&vs_forest::trees))
            .def_property_readonly("size", [](const vs_forest& self){ return self.trees().size(); })
            .def("__getitem__", [](vs_forest& self, unsigned int idx) -> vs_tree&
            {
                if(idx >= self.trees().size())
                {
                    throw py::index_error("index > tree numbers");
                }

                auto ptr = self.trees().begin();
                std::advance(ptr, idx);
                return *ptr;
            }, py::return_value_policy::reference)
            .def("__setitem__", [](vs_forest& self, unsigned int idx, const vs_tree& t)
            {
                if(idx >= self.trees().size())
                {
                    throw py::index_error("index > tree numbers");
                }

                auto ptr = self.trees().begin();
                std::advance(ptr, idx);
                *ptr = t;
            });



    /****************************************************
     *                      Settings                    *
     ****************************************************/
    py::enum_<vs::grow_func>(m, "DomainScaling", py::arithmetic())
            .value("NONE", vs::grow_func::none)
            .value("LINEAR", vs::grow_func::linear)
            .value("EXPONENTIAL", vs::grow_func::exponential);

    py::class_<vs::settings>(m, "Settings")
            .def(py::init<>())
            .def_readwrite("steps", &vs::settings::m_steps)
            .def_readwrite("samples", &vs::settings::m_sample_count)
            .def("scale", &vs::settings::scale)
            .def("system", &vs::settings::get_system_data, py::return_value_policy::reference);

    py::class_<vs::settings::system>(m, "SystemSettings")
            .def(py::init<>())
            .def_readwrite("parent_inertia", &vs::settings::system::m_parent_inertia)
            .def_readwrite("birth_attr", &vs::settings::system::m_birth_attr)
            .def_readwrite("birth_node", &vs::settings::system::m_birth_node)
            .def_readwrite("influence_attr", &vs::settings::system::m_influence_attr)
            .def_readwrite("kill_attr", &vs::settings::system::m_kill_attr)
            .def_readwrite("perception", &vs::settings::system::m_percept_vol)
            .def_readwrite("terminal_radius", &vs::settings::system::m_term_radius)
            .def_readwrite("terminal_growth", &vs::settings::system::m_growth_distance)
            .def_readwrite("bifurcation_thresh", &vs::settings::system::m_bif_thresh)
            .def_readwrite("bifurcation_index", &vs::settings::system::m_bif_index)
            .def_readwrite("only_leaf_growth", &vs::settings::system::m_only_leaf_development)
            .def_property("domain_scaling",
                          [](const vs::settings::system& self){ return self.m_grow_func.m_type; },
                          [](vs::settings::system& self, const vs::grow_func f){ self.m_grow_func.m_type = f; })
            .def_property("domain_scaling_value",
                          [](const vs::settings::system& self){ return self.m_grow_func.m_value; },
                          [](vs::settings::system& self, float val){ self.m_grow_func.m_value = val; });


    /****************************************************
     *             Vessel Tree Synthesizer              *
     ****************************************************/
    py::enum_<vs::system>(m, "System", py::arithmetic())
            .value("ARTERIAL", vs::system::arterial)
            .value("VENOUS", vs::system::venous);

    py::class_<vs::synthesizer>(m, "Synthesizer")
            .def(py::init<vs::domain&>())
            .def("create_root", &vs::synthesizer::create_root)
            .def_property("settings", &vs::synthesizer::get_settings, &vs::synthesizer::set_settings)
            .def("run", &vs::synthesizer::run)
            .def("init", &vs::synthesizer::init_runtime_params)
            .def("step_by_step", &vs::synthesizer::step_by_step)
            .def("get_arterial_forest", [](vs::synthesizer& self) { return self.get_forest(vs::system::arterial); }, py::return_value_policy::copy)
            .def("get_venous_forest", [](vs::synthesizer& self) { return self.get_forest(vs::system::venous); }, py::return_value_policy::copy)
            .def("set_arterial_forest",  [](vs::synthesizer& self, const vs::synthesizer::forest& trees) { return self.set_forest(vs::system::arterial, trees); })
            .def("set_venous_forest", [](vs::synthesizer& self, const vs::synthesizer::forest& trees) { return self.set_forest(vs::system::venous, trees); })
            .def("get_arterial_perftimes", [](vs::synthesizer& self) { return self.get_system_data(vs::system::arterial).m_profiler.get_samples(); })
            .def("get_venous_perftimes", [](vs::synthesizer& self) { return self.get_system_data(vs::system::venous).m_profiler.get_samples(); });


    /****************************************************
     *                    Profiler                      *
     ****************************************************/
    py::class_<vs::prf::monitor>(m, "PerfMonitor")
            .def_readonly_static("Enabled", &vs::prf::monitor::is_enabled);

    m.attr("not_a_node") = vs::not_a_node;

}