#pragma once

#include "binarytree.h"

namespace vs
{

template<typename Data = std::monostate>
struct forest
{
    using tree = binary_tree<Data>;

private:
    std::list<tree> m_trees;

public:
    template<typename... Args>
    tree& emplace_back(Args&&... args)
    {
        return m_trees.emplace_back(args...);
    }

    void erase(tree& tree)
    {
        std::erase_if(m_trees.begin(), m_trees.end(), [&](auto& list_tree){ &list_tree == &tree; });
    }

    std::list<tree>& trees()
    {
        return m_trees;
    }

    const std::list<tree>& trees() const
    {
        return m_trees;
    }

    void clear()
    {
        m_trees.clear();
    }

    template<typename Func>
    void breadth_first(const Func& func)
    {
        for(auto& t : m_trees)
        {
            t.breadth_first([&t, &func](auto& n){ func(t, n); }); /* TODO: had issues with std::bind */
        }
    }

    template<typename Func>
    void depth_first(const Func& func)
    {
        for(auto& t : m_trees)
        {
            t.depth_first([&t, &func](auto& n){ func(t, n); }); /* TODO: had issues with std::bind */
        }
    }

    template<typename Func>
    void post_order(const Func& func)
    {
        for(auto& t : m_trees)
        {
            t.post_order([&t, &func](auto& n){ func(t, n); }); /* TODO: had issues with std::bind */
        }
    }

    template<typename Func>
    void for_each(const Func& func)
    {
        for(auto& tree : m_trees)
        {
            func(tree);
        }
    }

    template<typename Func>
    void delete_if(const Func& func)
    {
        for(auto& t : m_trees)
        {
            t.delete_if([&t, &func](auto& n){ return func(t, n); }); /* TODO: had issues with std::bind */
        }
    }
};

}
