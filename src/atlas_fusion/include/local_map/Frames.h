/*
 * Copyright 2020 Brno University of Technology
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom
 * the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT
 * OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
 * OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#pragma once

#include <iostream>
#include <string>

namespace AtlasFusion::LocalMap {

    /**
     * Frames defines the names of the sensor's local coordinates systems.
     */
    namespace Frames {
        const std::string kOrigin = "origin";

        const std::string kLidarLeft = "lidar_left";
        const std::string kLidarRight = "lidar_right";
        const std::string kLidarCenter = "lidar_center";

        const std::string kRadarTi = "radar_ti";

        const std::string kImuFrame = "imu";

        const std::string kCameraLeftFront = "camera_left_front";
        const std::string kCameraLeftSide = "camera_left_side";
        const std::string kCameraRightFront = "camera_right_front";
        const std::string kCameraRightSide = "camera_right_side";
        const std::string kCameraIr = "camera_ir";
        const std::string kCameraVirtual = "camera_virtual";

        const std::string kGnssAntennaFront = "gnss_front";
        const std::string kGnssAntennaRear = "gnss_rear";

        const std::string kErr = "ERROR";

    }
}