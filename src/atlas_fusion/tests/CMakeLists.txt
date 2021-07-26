cmake_minimum_required(VERSION 3.13)
project(basic_test)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")

find_package(GTest REQUIRED)
enable_testing()

set(all_libs
        GTest::GTest
        GTest::Main
        atlas_fusion_srcs
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${PCL_LIBRARIES}
        stdc++fs
        yaml-cpp)

add_executable(data_loader_imu_test data_loader_imu_test.cpp)
target_compile_definitions(data_loader_imu_test PUBLIC TEST_FOLDER="${PROJECT_SOURCE_DIR}/")
target_link_libraries(data_loader_imu_test ${all_libs})
add_test(NAME data_loader_imu_test COMMAND data_loader_imu_test WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})


add_executable(data_loader_gnss_test data_loader_gnss_test.cpp)
target_compile_definitions(data_loader_gnss_test PUBLIC TEST_FOLDER="${PROJECT_SOURCE_DIR}/")
target_link_libraries(data_loader_gnss_test ${all_libs})
add_test(NAME data_loader_gnss_test COMMAND data_loader_gnss_test WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})


add_executable(data_loader_lidar_test data_loader_lidar_test.cpp)
target_compile_definitions(data_loader_lidar_test PUBLIC TEST_FOLDER="${PROJECT_SOURCE_DIR}/")
target_link_libraries(data_loader_lidar_test ${all_libs})
add_test(NAME data_loader_lidar_test COMMAND data_loader_lidar_test WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})


add_executable(data_loader_camera_test data_loader_camera_test.cpp)
target_compile_definitions(data_loader_camera_test PUBLIC TEST_FOLDER="${PROJECT_SOURCE_DIR}/")
target_link_libraries(data_loader_camera_test ${all_libs})
add_test(NAME data_loader_camera_test COMMAND data_loader_camera_test WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})


add_executable(depth_map_test depth_map_test.cpp)
target_compile_definitions(depth_map_test PUBLIC TEST_FOLDER="${PROJECT_SOURCE_DIR}/")
target_link_libraries(depth_map_test ${all_libs})
add_test(NAME depth_map_test COMMAND depth_map_test WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})


add_executable(kalman_1d_test kalman_1d_test.cpp)
target_compile_definitions(kalman_1d_test PUBLIC TEST_FOLDER="${PROJECT_SOURCE_DIR}/")
target_link_libraries(kalman_1d_test ${all_libs})
add_test(NAME kalman_1d_test COMMAND kalman_1d_test WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})


add_executable(kalman_3d_test kalman_3d_test.cpp)
target_compile_definitions(kalman_3d_test PUBLIC TEST_FOLDER="${PROJECT_SOURCE_DIR}/")
target_link_libraries(kalman_3d_test ${all_libs})
add_test(NAME kalman_3d_test COMMAND kalman_3d_test WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})


add_executable(laser_aggregator_test laser_aggregator_test.cpp)
target_compile_definitions(laser_aggregator_test PUBLIC TEST_FOLDER="${PROJECT_SOURCE_DIR}/")
target_link_libraries(laser_aggregator_test ${all_libs})
add_test(NAME laser_aggregator_test COMMAND laser_aggregator_test WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})


add_executable(local_pose_test local_pose_test.cpp)
target_compile_definitions(local_pose_test PUBLIC TEST_FOLDER="${PROJECT_SOURCE_DIR}/")
target_link_libraries(local_pose_test ${all_libs})
add_test(NAME local_pose_test COMMAND local_pose_test WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})


add_executable(pointcloud_aggregation_test pointcloud_aggregation_test.cpp)
target_compile_definitions(pointcloud_aggregation_test PUBLIC TEST_FOLDER="${PROJECT_SOURCE_DIR}/")
target_link_libraries(pointcloud_aggregation_test ${all_libs})
add_test(NAME pointcloud_aggregation_test COMMAND pointcloud_aggregation_test WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})


add_executable(pointcloud_extrapolation_test pointcloud_extrapolation_test.cpp)
target_compile_definitions(pointcloud_extrapolation_test PUBLIC TEST_FOLDER="${PROJECT_SOURCE_DIR}/")
target_link_libraries(pointcloud_extrapolation_test ${all_libs})
add_test(NAME pointcloud_extrapolation_test COMMAND pointcloud_extrapolation_test WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})


add_executable(projector_test projector_test.cpp)
target_compile_definitions(projector_test PUBLIC TEST_FOLDER="${PROJECT_SOURCE_DIR}/")
target_link_libraries(projector_test ${all_libs})
add_test(NAME projector_test COMMAND projector_test WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})


add_executable(point_spherical_test point_spherical_test.cpp)
target_compile_definitions(point_spherical_test PUBLIC TEST_FOLDER="${PROJECT_SOURCE_DIR}/")
target_link_libraries(point_spherical_test ${all_libs})
add_test(NAME point_spherical_test COMMAND point_spherical_test WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
