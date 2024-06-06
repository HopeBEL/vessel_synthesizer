############################################
#          Blood Vessel Generator          #
#------------------------------------------#
# @author  Nikolaus Rauch                  #
# @date    03.08.2020                      #
#------------------------------------------#
############################################
cmake_minimum_required( VERSION 3.9 )
project( vessel-synthesizer )

message(STATUS "${PROJECT_NAME} build environment --")
message(STATUS "Build system:     ${CMAKE_SYSTEM_NAME}-${CMAKE_HOST_SYSTEM_PROCESSOR}")

#########################################
#                Options                #
#########################################
option(VS_GOOGLE_TESTS "Build Google Test Programs" OFF)
option(VS_PYTHON_BINDINGS "Build Python Bindings" ON)
option(VS_PROFILER "Build with Profiler Functionality" ON)
option(VS_COMPILE_NATIVE "compile for micro-architecture and ISA extensions of the host" OFF)
option(VS_COMPILE_FASTMATH "compile with fastmath optimization" OFF)

#########################################
#              CMake-Stuff              #
#########################################
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin)


#########################################
#        Global Compiler Options        #
#########################################
set(GCC_COMPILE_OPTIONS "-Wall;-Wno-deprecated-enum-enum-conversion")
set(GCC_COMPILE_DEBUG_OPTIONS "${GCC_COMPILE_OPTIONS};-ggdb;-O0")
set(GCC_COMPILE_RELEASE_OPTIONS "${GCC_COMPILE_OPTIONS};-O3")

set(MSVC_COMPILE_OPTIONS "/W3")
set(MSVC_COMPILE_DEBUG_OPTIONS "${MSVC_COMPILE_OPTIONS};/Zi")
set(MSVC_COMPILE_RELEASE_OPTIONS "${MSVC_COMPILE_OPTIONS};/O2")
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

if(VS_COMPILE_NATIVE)
    set(GCC_COMPILE_RELEASE_OPTIONS "${GCC_COMPILE_RELEASE_OPTIONS};-march=native")
    set(MSVC_COMPILE_RELEASE_OPTIONS "${MSVC_COMPILE_RELEASE_OPTIONS};/arch")
endif()

if(VS_COMPILE_FASTMATH)
    set(GCC_COMPILE_RELEASE_OPTIONS "${GCC_COMPILE_RELEASE_OPTIONS};-ffast-math")
    set(MSVC_COMPILE_RELEASE_OPTIONS "${MSVC_COMPILE_RELEASE_OPTIONS};/fp:fast")
endif()

add_compile_options("$<$<AND:$<CXX_COMPILER_ID:GNU>,$<CONFIG:DEBUG>>:${GCC_COMPILE_DEBUG_OPTIONS}>")
add_compile_options("$<$<AND:$<CXX_COMPILER_ID:GNU>,$<CONFIG:RELEASE>>:${GCC_COMPILE_RELEASE_OPTIONS}>")

add_compile_options("$<$<AND:$<CXX_COMPILER_ID:MSVC>,$<CONFIG:DEBUG>>:${MSVC_COMPILE_DEBUG_OPTIONS}>")
add_compile_options("$<$<AND:$<CXX_COMPILER_ID:MSVC>,$<CONFIG:RELEASE>>:${MSVC_COMPILE_RELEASE_OPTIONS}>")


#########################################
#    Build Vessel Synthesis Methods     #
#########################################
add_subdirectory(vessel_synthesis)

#########################################
#             Python Bindings           #
#########################################
if(VS_PYTHON_BINDINGS)
    message(STATUS "Build Python Vessel-Synthesis Module")
    add_subdirectory(vessel_module)
endif(VS_PYTHON_BINDINGS)

