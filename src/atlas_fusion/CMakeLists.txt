cmake_minimum_required(VERSION 3.13)
project(atlas_fusion)

SET(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG " ${CMAKE_CXX_FLAGS_DEBUG} -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

## Find catkin and any catkin packages
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs tf2 tf2_msgs tf2_ros tf2_geometry_msgs image_transport cv_bridge)
find_package(Qt5 COMPONENTS Core REQUIRED)
find_package(PCL 1.7 REQUIRED COMPONENTS)
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

add_subdirectory(libs/robotic-template-library)
include_directories(
        libs/robotic-template-library/include
        libs/munkres/include
)


set(SOURCES
        src/MapBuilder.cpp
        src/LogService.cpp
        src/Context.cpp
        src/DataCache.cpp)

set(SOURCES_ALGORITHMS
        src/algorithms/SimpleTrajectoryLogger.cpp
        src/algorithms/Kalman1D.cpp
        src/algorithms/Kalman3D.cpp
        src/algorithms/ImuDataProcessor.cpp
        src/algorithms/LidarFilter.cpp
        src/algorithms/Projector.cpp
        src/algorithms/DepthMap.cpp
        src/algorithms/DetectionsProcessor.cpp
        src/algorithms/pointcloud/PointCloudExtrapolator.cpp
        src/algorithms/pointcloud/PointCloudAggregator.cpp
        src/algorithms/pointcloud/PointCloudProcessor.cpp
        src/algorithms/pointcloud/LaserAggregator.cpp
        src/algorithms/pointcloud/LaserSegmenter.cpp
        src/algorithms/pointcloud/GlobalPointcloudStorage.cpp
        src/algorithms/pointcloud/OccupancyGrid3D.cpp
        src/algorithms/yolo_reprojection/YoloDetectionReprojector.cpp
        src/algorithms/pointcloud/GlobalPointcloudStorage.cpp
        src/algorithms/pointcloud/ObjectDetector.cpp
        src/algorithms/SelfModel.cpp
        src/algorithms/image_processing/SimpleImageHandler.cpp

        )

set(SOURCES_LOCAL_MAP
        src/local_map/TFTree.cpp
        src/local_map/LocalMap.cpp
        src/local_map/ObjectsAggregator.cpp)

set(SOURCES_VISUALIZERS
        src/visualizers/VisualizationHandler.cpp
        src/visualizers/LidarVisualizer.cpp
        src/visualizers/ImuVisualizer.cpp
        src/visualizers/CameraVisualizer.cpp
        src/visualizers/GnssVisualizer.cpp
        src/visualizers/TrajectoryVisualizer.cpp
        src/visualizers/TFVisualizer.cpp
        src/visualizers/FrustumVisualizer.cpp
        src/visualizers/TelemetryVisualizer.cpp
        src/visualizers/RadarVisualizer.cpp)

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
        src/data_models/radar/RadarTiDataModel.cpp
        src/data_models/camera/CameraFrameDataModel.cpp
        src/data_models/camera/CameraIrFrameDataModel.cpp
        src/data_models/camera/DepthMapDataModel.cpp
        src/data_models/yolo/YoloDetectionClass.cpp
        src/data_models/local_map/LocalPosition.cpp
        src/data_models/local_map/PointCloudBatch.cpp
        src/data_models/local_map/LidarDetection.cpp
        src/data_models/local_map/Object.cpp)

set(SOURCES_DATA_LOADER
        src/data_loader/DataLoader.cpp
        src/data_loader/AbstractDataLoader.cpp
        src/data_loader/CameraDataLoader.cpp
        src/data_loader/LidarDataLoader.cpp
        src/data_loader/GnssDataLoader.cpp
        src/data_loader/ImuDataLoader.cpp
        src/data_loader/RadarTiDataLoader.cpp )

set(SOURCES_DATA_WRITERS
        src/data_writers/YoloDetectionWriter.cpp
        src/data_writers/Lidar2ImagePlotter.cpp
        src/data_writers/ImageWriter.cpp)

set(SOURCES_FAIL_CHECK
        src/fail_check/AbstrackFailChecker.cpp
        src/fail_check/FailChecker.cpp
        src/fail_check/CameraRGBFailChecker.cpp
        src/fail_check/CameraIrFailChecker.cpp
        src/fail_check/LidarFailChecker.cpp
        src/fail_check/ImuFailChecker.cpp
        src/fail_check/GnssFailChecker.cpp
        src/fail_check/RadarTiFileChacker.cpp src/algorithms/image_processing/SimpleImageHandler.cpp include/algorithms/image_processing/SimpleImageHandler.h)

add_library(atlas_fusion_srcs STATIC ${SOURCES} ${SOURCES_ALGORITHMS} ${SOURCES_LOCAL_MAP} ${SOURCES_VISUALIZERS} ${SOURCES_DATA_MODELS} ${SOURCES_DATA_LOADER} ${SOURCES_DATA_WRITERS} ${SOURCES_FAIL_CHECK})

add_executable(atlas_fusion src/main.cpp)
target_link_libraries(${PROJECT_NAME}
        atlas_fusion_srcs
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${PCL_LIBRARIES}
        Qt5::Core
        stdc++fs
        yaml-cpp)

add_subdirectory(tests)