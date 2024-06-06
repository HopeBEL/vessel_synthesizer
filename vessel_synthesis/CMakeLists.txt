set( VESSEL_SRC
    "${CMAKE_CURRENT_SOURCE_DIR}/domain.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/synthesizer.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/profiler.cpp"
    )

set( VESSEL_HDR
    "${CMAKE_CURRENT_SOURCE_DIR}/binarytree.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/forest.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/domain.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/octree.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/points.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/synthesizer.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/profiler.h"
    )

source_group( TREE ${CMAKE_CURRENT_SOURCE_DIR}
    FILES ${VESSEL_SRC} ${VESSEL_HDR} )


#########################################
#        Build External-Libraries       #
#########################################
add_subdirectory(external/glm)
set_property(TARGET glm_static APPEND_STRING PROPERTY COMPILE_FLAGS " -w")
target_include_directories( glm_static PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/external>
    )

add_subdirectory(external/eigen)


#################################
#       Build Vessel Library    #
#################################
add_library( vessel_lib STATIC ${VESSEL_SRC} ${VESSEL_HDR} )

target_link_libraries( vessel_lib PUBLIC glm_static eigen )

target_include_directories( vessel_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
    )

target_compile_features( vessel_lib PUBLIC cxx_std_20 )
set_target_properties( vessel_lib PROPERTIES CXX_EXTENSIONS OFF )


if(VS_PROFILER)
    message(STATUS "Build with Profiling!")
    target_compile_definitions(vessel_lib PUBLIC VS_PROFILER)
endif(VS_PROFILER)

#########################################
#           Build Google Tests          #
#########################################
if(VS_GOOGLE_TESTS)
    message(STATUS "Build Google Tests for Vessel-Synthesis!")
    add_subdirectory(test)
endif(VS_GOOGLE_TESTS)
