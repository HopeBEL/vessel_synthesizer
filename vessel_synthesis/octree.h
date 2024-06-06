#pragma once

#include <array>
#include <vector>
#include <map>

namespace vs::util
{

namespace detail
{

constexpr static int pow( int base, int exp ) noexcept
{
    return (exp == 0 ? 1 : base * pow( base, exp - 1) );
}

}


template<typename Point, int N, typename Data>
struct oc_tree
{
    /*****************************************************/
    /*                        Node                       */
    /*****************************************************/
    struct node
    {
        Point m_min;
        Point m_max;
        Point m_center;

        int m_max_pop;
        int m_depth;

    public:
        node(const Point& min, const Point& max, unsigned int max_pop, unsigned int depth)
            : m_min(min), m_max(max), m_max_pop(max_pop), m_depth(depth)
        {
            m_center = (max + min) * 0.5f;
        }

        virtual ~node() = default;

        virtual bool is_leaf() const noexcept { return false; }

        virtual bool insert( const Point& p, const Data& data ) noexcept = 0;
        virtual bool remove( const Point& p, const Data& data ) noexcept = 0;

        virtual void euclidean_range( const Point& p, float range, std::multimap<float, Data>& result) const noexcept = 0;
        virtual void euclidean_range( const Point& p, float range, std::vector<Data>& result) const noexcept = 0;
    };

    /*****************************************************/
    /*                       Branch                      */
    /*****************************************************/
    struct branch : public node
    {
        std::array<node*, detail::pow(2, N)> m_children;

    public:
        branch(const Point& min, const Point& max, unsigned int max_pop, unsigned int depth)
            : node(min, max, max_pop, depth)
        {
            std::fill(m_children.begin(), m_children.end(), nullptr);
        }

        virtual ~branch()
        {
            std::for_each(m_children.begin(), m_children.end(), [](auto* child) { if(child) { delete child; } });
        }

        virtual bool insert( const Point& p, const Data& data ) noexcept override
        {
            int index = 0;
            Point min;
            Point max;

            for(int i = 0; i < N; i++)
            {
                if( p[i] > node::m_center[i] )
                {
                    index += (1 << i);
                    min[i] = node::m_center[i];
                    max[i] = node::m_max[i];
                }
                else
                {
                    min[i] = node::m_min[i];
                    max[i] = node::m_center[i];
                }
            }

            if( m_children[index] )
            {
                if( !m_children[index]->insert(p, data) )
                {
                    auto* tmb_b = new branch( m_children[index]->m_min, m_children[index]->m_max, m_children[index]->m_max_pop, node::m_depth + 1 );
                    auto* tmb_l = static_cast<leaf*>( m_children[index] );

                    for( int i = 0; i < static_cast<int>(tmb_l->m_data.size()); i++ )
                    {
                        tmb_b->insert( tmb_l->m_point[i], tmb_l->m_data[i] );
                    }

                    tmb_b->insert(p, data);

                    delete tmb_l;
                    m_children[index] = tmb_b;

                    return true;
                }
            }
            else
            {
                m_children[index] = new leaf( min, max, node::m_max_pop, node::m_depth + 1 );
                m_children[index]->insert( p, data );
            }

            return true;
        }


        virtual bool remove( const Point& p, const Data& data ) noexcept override
        {
            int index = 0;
            Point min;
            Point max;

            for(int i = 0; i < N; i++)
            {
                if( p[i] > node::m_center[i] )
                {
                    index += (1 << i);
                    min[i] = node::m_center[i];
                    max[i] = node::m_max[i];
                }
                else
                {
                    min[i] = node::m_min[i];
                    max[i] = node::m_center[i];
                }
            }

            if( m_children[index] )
            {
                m_children[index]->remove(p, data);
            }

            return false;
        }

        virtual void euclidean_range( const Point& p, float range, std::multimap<float, Data>& result) const noexcept override
        {
            for( int i = 0; i < N; i++ )
            {
                if( node::m_max[i] < (p[i] - range) || node::m_min[i] >= (p[i] + range) )
                {
                    return;
                }
            }

            for(auto* child : m_children)
            {
                if(child) child->euclidean_range( p, range, result );
            }
        }

        virtual void euclidean_range( const Point& p, float range, std::vector<Data>& result) const noexcept override
        {
            for( int i = 0; i < N; i++ )
            {
                if( node::m_max[i] < (p[i] - range) || node::m_min[i] >= (p[i] + range) )
                {
                    return;
                }
            }

            for(auto* child : m_children)
            {
                if(child) child->euclidean_range( p, range, result );
            }
        }
    };


    /*****************************************************/
    /*                        Leaf                       */
    /*****************************************************/
    struct leaf : public node
    {
        std::vector<Data> m_data;
        std::vector<Point> m_point;

    public:
        leaf(const Point& min, const Point& max, unsigned int max_pop, unsigned int depth)
            : node(min, max, max_pop, depth)
        {
            m_data.reserve(max_pop);
            m_data.reserve(max_pop);
        }

        virtual ~leaf() = default;

        bool is_leaf() const noexcept override { return true; }

        virtual bool insert( const Point& p, const Data& data ) noexcept override
        {
            if( m_data.size() < m_data.capacity() )
            {
                m_data.emplace_back(data);
                m_point.emplace_back(p);
                return true;
            }

            return false;
        }

        virtual bool remove( const Point& p, const Data& data ) noexcept override
        {
            (void) p;
            auto iter = std::find(m_data.begin(), m_data.end(), data);

            if( iter != m_data.end() )
            {
                m_data.erase(iter);
                m_point.erase( m_point.begin() + std::distance(m_data.begin(), iter) );
                return true;
            }

            return false;
        }

        virtual void euclidean_range( const Point& p, float range, std::multimap<float, Data>& result) const noexcept override
        {
            for( int i = 0; i < N; i++ )
            {
                if( node::m_max[i] < p[i] - range || node::m_min[i] >= p[i] + range )
                {
                    return;
                }
            }

            for(int j = 0; j < static_cast<int>(m_data.size()); j++)
            {
                float distance = 0.0f;
                for(int i = 0; i < N; i++)
                {
                    float d = p[i] - m_point[j][i];
                    distance += d*d;
                }

                if( distance <= range*range )
                {
                    result.emplace(distance, m_data[j]);
                }
            }
        }

        virtual void euclidean_range( const Point& p, float range, std::vector<Data>& result) const noexcept override
        {
            for( int i = 0; i < N; i++ )
            {
                if( node::m_max[i] < p[i] - range || node::m_min[i] >= p[i] + range )
                {
                    return;
                }
            }

            for(int j = 0; j < static_cast<int>(m_data.size()); j++)
            {
                float distance = 0.0f;
                for(int i = 0; i < N; i++)
                {
                    float d = p[i] - m_point[j][i];
                    distance += d*d;
                }

                if( distance <= range*range )
                {
                    result.emplace_back(m_data[j]);
                }
            }
        }
    };



private:
    Point m_min;
    Point m_max;
    int m_max_pop;

    node* m_root{nullptr};

public:
    oc_tree(const Point& min, const Point& max, unsigned int max_pop)
        : m_min(min), m_max(max), m_max_pop(max_pop), m_root(nullptr)
    {
        m_root = new leaf( m_min, m_max, m_max_pop, 1 );
    }

    ~oc_tree()
    {
        if(m_root) { delete m_root; }
    }

    void clear()
    {
        delete m_root;
        m_root = new leaf( m_min, m_max, m_max_pop, 1 );
    }

    bool insert( const Point& p, const Data& data ) noexcept
    {
        for( int i = 0; i < N; i++ )
        {
            if(p[i] < m_min[i] || p[i] > m_max[i])
            {
                return false;
            }
        }

        if( m_root->insert(p, data) ) return true;

        auto* tmp_b = new branch( m_min, m_max, m_max_pop, 1 );
        auto* tmp_l = static_cast<leaf*>( m_root );

        for(int i = 0; i < static_cast<int>(tmp_l->m_data.size()); i++)
        {
            tmp_b->insert(tmp_l->m_point[i], tmp_l->m_data[i]);
        }

        tmp_b->insert(p, data);

        delete tmp_l;
        m_root = tmp_b;

        return true;
    }

    bool remove( const Point& p, const Data& data ) noexcept
    {
        for( int i = 0; i < N; i++ )
        {
            if(p[i] < m_min[i] || p[i] > m_max[i])
            {
                return false;
            }
        }

        return m_root->remove(p, data);
    }

    void euclidean_range( const Point& p, float range, std::multimap<float, Data>& result) const noexcept
    {
        result.clear();
        m_root->euclidean_range(p, range, result);
    }

    void euclidean_range( const Point& p, float range, std::vector<Data>& result) const noexcept
    {
        result.clear();
        m_root->euclidean_range(p, range, result);
    }

    template<typename Func>
    void traverse(const Func& func)
    {
        visit(m_root, func);
    }

    template<typename Func>
    void visit(node* curr_node, const Func& func)
    {
        if(curr_node->is_leaf())
        {
            auto* _leaf = static_cast<leaf*>(curr_node);
            for(const auto& data : _leaf->m_data)
            {
                func(data);
            }
        }
        else
        {
            auto* _branch = static_cast<branch*>(curr_node);
            for(auto* c : _branch->m_children)
            {
                if(c) { visit(c, func); }
            }
        }
    }
};

}
