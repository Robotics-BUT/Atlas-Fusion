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


        const std::string kCameraLeftFront = "/autodrive/local_map/cameras/camera_left_front/camera";
        const std::string kCameraLeftSide = "/autodrive/local_map/cameras/camera_left_side/camera";
        const std::string kCameraRightFront = "/autodrive/local_map/cameras/camera_right_front/camera";
        const std::string kCameraRightSide = "/autodrive/local_map/cameras/camera_right_side/camera";
        const std::string kCameraIr = "/autodrive/local_map/cameras/camera_ir/camera";

        const std::string kCameraLeftFrontInfo = "/autodrive/local_map/cameras/camera_left_front/camera_info";
        const std::string kCameraLeftSideInfo = "/autodrive/local_map/cameras/camera_left_side/camera_info";
        const std::string kCameraRightFrontInfo = "/autodrive/local_map/cameras/camera_right_front/camera_info";
        const std::string kCameraRightSideInfo = "/autodrive/local_map/cameras/camera_right_side/camera_info";
        const std::string kCameraIrInfo = "/autodrive/local_map/cameras/camera_ir/camera_info";


        const std::string kRawTrajectory = "/autodrive/local_map/trajectory/raw";
        const std::string kFilteredTrajectory = "/autodrive/local_map/trajectory/filtered";
        const std::string kImuGpsTrajectory = "/autodrive/local_map/trajectory/imu_gps";

        const std::string kYoloFrustumDetections = "/autodrive/local_map/yolo/frustums";


        const std::string kLidarAggregated = "/autodrive/local_map/lidar/aggregated";
        const std::string kLidarLaser = "/autodrive/local_map/lidar/laser";
        const std::string kGlobalPointCloud = "/autodrive/local_map/lidar/global";

    }
}