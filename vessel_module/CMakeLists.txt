add_subdirectory(external/pybind11)

pybind11_add_module( vessel_module "${CMAKE_CURRENT_SOURCE_DIR}/python_bindings.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/glm_cast.h")
target_compile_features( vessel_module PUBLIC cxx_std_20 )
set_target_properties( vessel_module PROPERTIES CXX_EXTENSIONS OFF )
target_link_libraries( vessel_module PUBLIC vessel_lib )

# doesn't work with cxx_std_23 compile_feature in vessel_lib ?
# target_link_options( vessel_module PRIVATE -static-libgcc -static-libstdc++ -Wl,-Bstatic,--whole-archive -lwinpthread -Wl,--no-whole-archive )
