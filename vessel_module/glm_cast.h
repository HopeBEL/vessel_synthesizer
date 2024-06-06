#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

#include <glm/gtc/type_ptr.hpp>

namespace PYBIND11_NAMESPACE { namespace detail {

template<typename T, glm::precision P>
struct type_caster<glm::tvec3<T, P>>
{
    using vector_type = glm::tvec3<T, P>;
    using Scalar = T;
    static constexpr std::size_t num_elements = 3;

    PYBIND11_TYPE_CASTER(vector_type, const_name("vec3"));

    bool load(handle src, bool)
    {
        array_t<Scalar> buf(src, true);
        if (!buf.check())
        {
            return false;
        }

        if (buf.ndim() == 1) // a 1-dimensional vector
        {
            if (buf.shape(0) != num_elements) {
                return false; // not a 3-elements vector
            }

            if (buf.strides(0) != sizeof(Scalar))
            {
                return false;
            }

            value = glm::make_vec3(buf.mutable_data()); // make_vec* copies the data (unnecessarily)
        }
        else
        {
            // buf.ndim() != 1
            return false;
        }
        return true;
    }

    static handle cast(const vector_type& src, return_value_policy /* policy */, handle /* parent */)
    {
        return array(
            num_elements,			// shape
            glm::value_ptr(src)		// data
        ).release();
    }
};

}}
