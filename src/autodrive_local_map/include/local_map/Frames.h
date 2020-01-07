#pragma once

#include <iostream>
#include <string>

namespace AutoDrive::LocalMap {

    namespace Frames {
        const std::string kOrigin = "origin";

        const std::string kLidarLeft = "lidar_left";
        const std::string kLidarRight = "lidar_right";

        const std::string kImuFrame = "imu";

        const std::string kCameraLeftFront = "camera_left_front";
        const std::string kCameraLeftSide = "camera_left_side";
        const std::string kCameraRightFront = "camera_right_front";
        const std::string kCameraRightSide = "camera_right_side";
        const std::string kCameraIr = "camera_ir";

        const std::string kGnssAntennaFront = "gnss_front";
        const std::string kGnssAntennaRear = "gnss_rear";

    }
}