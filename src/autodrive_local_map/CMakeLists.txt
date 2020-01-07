cmake_minimum_required(VERSION 3.16)
project(autodrive_local_map)

SET(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG " ${CMAKE_CXX_FLAGS_DEBUG} -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

## Find catkin and any catkin packages
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs tf2 tf2_msgs tf2_ros tf2_geometry_msgs image_transport cv_bridge)
find_package(Qt5 COMPONENTS Core REQUIRED)
find_package(PCL 1.7 REQUIRED COMPONENTS common io)
find_package(Yaml-cpp)


## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${Qt5_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_subdirectory(robotic-template-library)
include_directories(
        robotic-template-library/include
)


set(SOURCES
        src/MapBuilder.cpp
        src/LogService.cpp)

set(SOURCES_ALGORITHMS
        src/algorithms/SimpleTrajectoryLogger.cpp
        src/algorithms/Kalman1D.cpp
        src/algorithms/Kalman3D.cpp
        src/algorithms/MovementModel.cpp
        src/algorithms/ImuDataProcessor.cpp
        src/algorithms/LidarFilter.cpp
        src/algorithms/Projector.cpp
        src/algorithms/DepthMap.cpp
        src/algorithms/DetectionsProcessor.cpp)

set(SOURCES_LOCAL_MAP
        src/local_map/TFTree.cpp
        src/local_map/LocalMap.cpp)

set(SOURCES_VISUALIZERS
        src/visualizers/VisualizationHandler.cpp
        src/visualizers/LidarVisualizer.cpp
        src/visualizers/ImuVisualizer.cpp
        src/visualizers/CameraVisualizer.cpp
        src/visualizers/GnssVisualizer.cpp
        src/visualizers/TrajectoryVisualizer.cpp
        src/visualizers/TFVisualizer.cpp
        src/visualizers/FrustumVisualizer.cpp)

set(SOURCES_DATA_MODELS
        src/data_models/gnss/GnssTimeDataModel.cpp
        src/data_models/gnss/GnssPoseDataModel.cpp
        src/data_models/imu/ImuDquatDataModel.cpp
        src/data_models/imu/ImuGnssDataModel.cpp
        src/data_models/imu/ImuImuDataModel.cpp
        src/data_models/imu/ImuMagDataModel.cpp
        src/data_models/imu/ImuPressureDataModel.cpp
        src/data_models/imu/ImuTempDataModel.cpp
        src/data_models/imu/ImuTimeDataModel.cpp
        src/data_models/lidar/LidarScanDataModel.cpp
        src/data_models/camera/CameraFrameDataModel.cpp
        src/data_models/camera/CameraIrFrameDataModel.cpp
        src/data_models/yolo/YoloDetectionClass.cpp)

set(SOURCES_DATA_LOADER
        src/data_loader/DataLoader.cpp
        src/data_loader/AbstractDataLoader.cpp
        src/data_loader/CameraDataLoader.cpp
        src/data_loader/LidarDataLoader.cpp
        src/data_loader/GnssDataLoader.cpp
        src/data_loader/ImuDataLoader.cpp
        )

add_library(autodrive_srcs STATIC ${SOURCES} ${SOURCES_ALGORITHMS} ${SOURCES_LOCAL_MAP} ${SOURCES_VISUALIZERS} ${SOURCES_DATA_MODELS} ${SOURCES_DATA_LOADER})

add_executable(autodrive_local_map src/main.cpp)
target_link_libraries(${PROJECT_NAME}
        autodrive_srcs
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${PCL_LIBRARIES}
        Qt5::Core
        rtl
        stdc++fs
        yaml-cpp)

add_subdirectory(tests)