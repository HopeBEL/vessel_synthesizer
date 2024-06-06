#pragma once

#include "binarytree.h"

#include <glm/gtx/norm.hpp>
#include <glm/vec3.hpp>

#include <variant>

namespace vs
{


/*****************************************************/
/*              Attraction Points                    */
/*---------------------------------------------------*/
/*  act as oxygen drains and carbon dioxide source   */
/*****************************************************/
template<typename Data = std::monostate>
struct attr_point
{
    glm::vec3 m_pos;
    Data m_data;

public:
    attr_point() = default;

    template<typename ...Args>
    attr_point(const glm::vec3& pos, Args&& ...args)
        : m_pos(pos), m_data(std::move(args)...)
    {

    }

    bool operator ==(const attr_point& rhs)
    {
        /* this is obviously not ideal, but i need a equal criterion for finding points in this oc-tree implementation */
        auto l2 = glm::distance2(m_pos, rhs.m_pos);
        return l2 < 1e-8;
    }
};

/*****************************************************/
/*                Vesse Node Data                    */
/*---------------------------------------------------*/
/* --> a small vessel segment (node)                 */
/*****************************************************/
struct node_data
{
    glm::vec3 m_pos;
    float m_radius;
    binary_tree<node_data>* m_tree; /* TODO: sorry for this ... i need the tree it belongs to */
};

}
