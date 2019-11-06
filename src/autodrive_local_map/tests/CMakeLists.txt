cmake_minimum_required(VERSION 3.16)
project(basic_test)

find_package(GTest REQUIRED)
enable_testing()

add_executable(generic_test generic_test.cpp)
target_link_libraries(generic_test
        GTest::GTest
        GTest::Main
        algorithms_srcs
        local_map_srcs
        visualizers_srcs
        data_loader_srcs
        ${OpenCV_LIBRARIES}
        ${PCL_LIBRARIES}
        stdc++fs)
add_test(NAME generic_test COMMAND generic_test WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})


add_executable(data_loader_imu_test data_loader_imu_test.cpp)
target_compile_definitions(data_loader_imu_test PUBLIC TEST_FOLDER="${PROJECT_SOURCE_DIR}/")
target_link_libraries(data_loader_imu_test
        GTest::GTest
        GTest::Main
        data_loader_srcs
        ${OpenCV_LIBRARIES}
        ${PCL_LIBRARIES}
        stdc++fs)
add_test(NAME data_loader_imu_test COMMAND data_loader_imu_test WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})


add_executable(data_loader_gnss_test data_loader_gnss_test.cpp)
target_compile_definitions(data_loader_gnss_test PUBLIC TEST_FOLDER="${PROJECT_SOURCE_DIR}/")
target_link_libraries(data_loader_gnss_test
        GTest::GTest
        GTest::Main
        data_loader_srcs
        ${OpenCV_LIBRARIES}
        ${PCL_LIBRARIES}
        stdc++fs)
add_test(NAME data_loader_gnss_test COMMAND data_loader_gnss_test WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})



add_executable(data_loader_lidar_test data_loader_lidar_test.cpp)
target_compile_definitions(data_loader_lidar_test PUBLIC TEST_FOLDER="${PROJECT_SOURCE_DIR}/")
target_link_libraries(data_loader_lidar_test
        GTest::GTest
        GTest::Main
        data_loader_srcs
        ${OpenCV_LIBRARIES}
        ${PCL_LIBRARIES}
        stdc++fs)
add_test(NAME data_loader_lidar_test COMMAND data_loader_lidar_test WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})


add_executable(data_loader_camera_test data_loader_camera_test.cpp)
target_compile_definitions(data_loader_camera_test PUBLIC TEST_FOLDER="${PROJECT_SOURCE_DIR}/")
target_link_libraries(data_loader_camera_test
        GTest::GTest
        GTest::Main
        data_loader_srcs
        ${OpenCV_LIBRARIES}
        ${PCL_LIBRARIES}
        stdc++fs)
add_test(NAME data_loader_camera_test COMMAND data_loader_camera_test WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})