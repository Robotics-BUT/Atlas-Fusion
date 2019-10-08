cmake_minimum_required(VERSION 3.1.0)
project(basic_test)

find_package(GTest REQUIRED)

enable_testing()

add_executable(generic_test generic_test.cpp)
target_link_libraries(generic_test
        GTest::GTest
        GTest::Main
        algorithms_srcs
        local_map_srcs
        visualizers_srcs )
add_test(generic_test Generic_Test)