#pragma once

#include <algorithm>
#include <array>
#include <cassert>
#include <limits>
#include <list>
#include <unordered_map>
#include <stack>
#include <queue>
#include <variant>
#include "domain.h"
#include <string>


namespace vs
{

typedef unsigned int node_id;
inline constexpr node_id not_a_node = std::numeric_limits<node_id>::max();

template<typename Data = std::monostate>
struct binary_tree
{
    struct node
    {
    private:
        node_id m_id{not_a_node};
        node_id m_parent{not_a_node};
        std::array<node_id, 4> m_children{not_a_node, not_a_node, not_a_node, not_a_node};
        Data m_data;

    public:
        template<typename ...Args>
        node(const node_id id, const node_id parent, Args&&... args)
            : m_id{id}, m_parent{parent}, m_data(args...) {}

        bool is_root() const { return m_parent == not_a_node; }
        bool is_leaf() const { return m_children[0] == not_a_node && m_children[1] == not_a_node; }
        bool is_inter() const { return !(m_children[0] == not_a_node) != !(m_children[1] == not_a_node); }
        bool is_joint() const { return m_children[0] != not_a_node && m_children[1] != not_a_node; }

        node_id id() const { return m_id; }
        node_id parent() const { return m_parent; }
        std::array<node_id, 4> children() const { return m_children; }

        Data& data() { return m_data; }
        const Data& data() const { return m_data; }

        operator node_id() { return m_id; }

        friend binary_tree;
    };


private:
    node_id m_next_id{0};
    node_id m_root_id{not_a_node};
    std::unordered_map<node_id, node> m_nodes;


public:
    template<typename ...Args>
    node& create_root(Args&&... args)
    {
        assert(m_root_id == not_a_node);

        m_root_id = m_next_id++;
        auto [iter, _] = m_nodes.emplace(m_root_id, node(m_root_id, not_a_node, std::move(args)...));
        return iter->second;
    }

    template<typename ...Args>
    node& create_node(const node_id parent, domain& domain_, Args&&... args)
    {
        assert(exists(parent));

        auto id = m_next_id++;
        domain_.m_logs += "Test create_node with parent : " + std::to_string(parent) + " and child id is now : " + std::to_string(id) + "\n";
        auto [iter, _] = m_nodes.emplace(id, node(id, parent, std::move(args)...));

        attach_node(parent, domain_, iter->second);

        return iter->second;
    }

    void delete_node(const node_id id)
    {
        if(!exists(id)) { return; }

        std::list<node_id> erase_list;
        breadth_first([&](const auto& node)
        {
            erase_list.emplace_back(node.id());
        }, id);

        detach_node(id);

        std::for_each(erase_list.begin(), erase_list.end(), [this](const auto id){ m_nodes.erase(id); });
    }

    node& get_root()
    {
        return get_node(m_root_id);
    }

    const node& get_root() const
    {
        return get_node(m_root_id);
    }

    node& get_node(const node_id id)
    {
        assert(exists(id));

        auto search = m_nodes.find(id);
        return (*search).second;
    }

    const node& get_node(const node_id id) const
    {
        assert(exists(id));

        auto search = m_nodes.find(id);
        return (*search).second;
    }

    node& operator[](const node_id id)
    {
        return get_node(id);
    }

    const node& operator[](const node_id id) const
    {
        return get_node(id);
    }

    template<typename Func>
    void breadth_first(const Func& func, node_id start = not_a_node)
    {
        auto _start = (start == not_a_node) ? m_root_id : start;
        if(!exists(_start)) { return; }

        std::queue<node_id> queue;
        queue.push(_start);

        while(!queue.empty())
        {
            auto curr_id = queue.front();
            queue.pop();

            auto& curr_node = get_node(curr_id);
            func(curr_node);

            auto children_ids = curr_node.children();
            if(children_ids[0] != not_a_node) queue.push(children_ids[0]);
            if(children_ids[1] != not_a_node) queue.push(children_ids[1]);
            if(children_ids[2] != not_a_node) queue.push(children_ids[2]);
            if(children_ids[3] != not_a_node) queue.push(children_ids[3]);
            
        }
    }

    template<typename Func>
    void depth_first(const Func& func, node_id start = not_a_node)
    {
        auto _start = (start == not_a_node) ? m_root_id : start;
        if(!exists(_start)) { return; }

        std::list<node_id> list;
        list.push_front(_start);

        while(!list.empty())
        {
            auto curr_id = list.front();
            list.pop_front();

            auto& curr_node = get_node(curr_id);
            func(curr_node);

            auto children_ids = curr_node.children();
            if(children_ids[1] != not_a_node) list.push_front(children_ids[1]);
            if(children_ids[0] != not_a_node) list.push_front(children_ids[0]);
        }
    }

    template<typename Func>
    void post_order(const Func& func, node_id start = not_a_node)
    {
        std::stack<node_id> s_1, s_2;

        auto _start = (start == not_a_node) ? m_root_id : start;
        if(!exists(_start)) { return; }
        s_1.push(_start);

        while(!s_1.empty())
        {
            auto curr_id = s_1.top();
            s_1.pop();
            s_2.push(curr_id);

            auto children_ids = get_node(curr_id).children();
            if(children_ids[0] != not_a_node) s_1.push(children_ids[0]);
            if(children_ids[1] != not_a_node) s_1.push(children_ids[1]);
        }

        while(!s_2.empty())
        {
            auto curr_id = s_2.top();
            s_2.pop();

            auto& curr_node = get_node(curr_id);
            func(curr_node);
        }
    }

    template<typename Func>
    void to_root(const Func& func, node_id start)
    {
        if(!exists(start)) { return; }

        auto curr_id = start;
        while(curr_id != not_a_node)
        {
            auto& curr_node = get_node(curr_id);
            func(curr_node);

            curr_id = curr_node.parent();
        }
    }

    template<typename Func>
    void delete_if(const Func& func, node_id start = not_a_node)
    {
        auto _start = (start == not_a_node) ? m_root_id : start;
        if(!exists(_start)) { return; }

        std::list<node_id> erase_list;
        breadth_first([&](auto& node)
        {
            if(func(node))
            {
                erase_list.emplace_back(node.id());
            }
        }, _start);

        std::for_each(erase_list.begin(), erase_list.end(), [this](const auto id){ delete_node(id); });
    }

    void clear()
    {
        m_nodes.clear();
        m_next_id = 0;
        m_root_id = not_a_node;
    }

    bool exists(const node_id id) const
    {
        return m_nodes.find(id) != m_nodes.end();
    }

    std::size_t size() const
    {
        return m_nodes.size();
    }

    std::unordered_map<node_id, node>& get_all_nodes()
    {
        return m_nodes;
    }

    const std::unordered_map<node_id, node>& get_all_nodes() const
    {
        return m_nodes;
    }

private:
    void detach_node(const node_id id)
    {
        auto& del_node = get_node(id);
        if(!del_node.is_root())
        {
            auto& parent_node = get_node(del_node.parent());
            if(parent_node.m_children[0] == id)
            {
                parent_node.m_children[0] = not_a_node;
                std::swap(parent_node.m_children[0], parent_node.m_children[1]);
            }
            else if(parent_node.m_children[1] == id)
            {
                parent_node.m_children[1] = not_a_node;
            }
        }
        del_node.m_parent = not_a_node;
    }

    void attach_node(const node_id parent, domain& domain_, node& child)
    {
        auto& parent_node = get_node(parent);
        if (parent == m_root_id) {
            domain_.m_logs += "Called attach from root\n";
            for (int i = 0; i < 4; i++) {
                if (parent_node.m_children[i] == not_a_node) {
                    parent_node.m_children[i] = child.id();
                    domain_.m_logs += "Attach_node : added child to " + std::to_string(parent) + " child id " + std::to_string(child.id()) + "\n"; 
                    return;
                }
            }
        }
        else {
            assert(!parent_node.is_joint());

            if(parent_node.m_children[0] == not_a_node) { parent_node.m_children[0] = child.id(); }
            else if(parent_node.m_children[1] == not_a_node) { parent_node.m_children[1] = child.id(); }
        }
    }
};

}
