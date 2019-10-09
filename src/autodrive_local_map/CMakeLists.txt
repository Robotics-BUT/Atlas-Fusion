cmake_minimum_required(VERSION 3.1.0)
project(autodrive_local_map)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

## Find catkin and any catkin packages
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
find_package(Qt5 COMPONENTS Core REQUIRED)



## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(
        include
        src/algorithms/include
        src/local_map/include
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${Qt5_INCLUDE_DIRS}
)

add_subdirectory(robotic-template-library)
include_directories(
        robotic-template-library/include
)


set(SOURCES
        src/MapBuilder.cpp)

set(SOURCES_ALGORITHMS
        src/algorithms/GenericAlgorithm.cpp)

set(SOURCES_LOCAL_MAP
        src/local_map/GenericLocalMap.cpp)

set(SOURCES_VISUALIZERS
        src/visualizers/GenericVisualizer.cpp)


add_library(basic_srcs STATIC ${SOURCES})
add_library(algorithms_srcs STATIC ${SOURCES_ALGORITHMS})
add_library(local_map_srcs STATIC ${SOURCES_LOCAL_MAP})
add_library(visualizers_srcs STATIC ${SOURCES_VISUALIZERS})

add_executable(autodrive_local_map src/main.cpp)
target_link_libraries(${PROJECT_NAME}
        basic_srcs
        algorithms_srcs
        local_map_srcs
        visualizers_srcs
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        Qt5::Core
        robotic-template-library)

add_subdirectory(tests)