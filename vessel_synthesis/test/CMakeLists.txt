
# https://google.github.io/googletest/quickstart-cmake.html

#################################
#        Get google tests       #
#################################
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/8aa657ee397a3273f9d2bb14a10b39fd1d556a77.zip
)

# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()

#################################
#            Build Tests        #
#################################
add_executable( vs_tests
    tree_test.cpp
)

target_link_libraries( vs_tests PRIVATE vessel_lib gtest_main gmock_main)
target_compile_features( vs_tests PUBLIC cxx_std_20 )
set_target_properties( vs_tests PROPERTIES CXX_EXTENSIONS OFF )

include(GoogleTest)
gtest_discover_tests(vs_tests)
