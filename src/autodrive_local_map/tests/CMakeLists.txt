cmake_minimum_required(VERSION 3.16)
project(basic_test)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")

find_package(GTest REQUIRED)
enable_testing()

set(all_libs
        GTest::GTest
        GTest::Main
        autodrive_srcs
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


add_executable(projector_test projector_test.cpp)
target_compile_definitions(projector_test PUBLIC TEST_FOLDER="${PROJECT_SOURCE_DIR}/")
target_link_libraries(projector_test ${all_libs})
add_test(NAME projector_test COMMAND projector_test WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})