#pragma once

#include <iostream>
#include <string>

namespace AutoDrive::Visualizers {

    namespace Topics {
        const std::string kTestCubeTopic = "/autodrive/local_map/test_cube";
        const std::string kLidarLeft = "/autodrive/local_map/lidar/left";
        const std::string kLidarRight = "/autodrive/local_map/lidar/right";
        const std::string kImuTopic = "/autodrive/local_map/imu/imu";
        const std::string kGnssTopic = "/autodrive/local_map/gnss/pose_text";


        const std::string kCameraLeftFront = "/autodrive/local_map/cameras/camera_left_front";
        const std::string kCameraLeftSide = "/autodrive/local_map/cameras/camera_left_side";
        const std::string kCameraRightFront = "/autodrive/local_map/cameras/camera_right_front";
        const std::string kCameraRightSide = "/autodrive/local_map/cameras/camera_right_side";
        const std::string kCameraIr = "/autodrive/local_map/cameras/camera_ir";
    }
}