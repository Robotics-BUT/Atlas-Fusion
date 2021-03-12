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

namespace AtlasFusion::Visualizers {

    namespace Topics {
        const std::string kTestCubeTopic = "/autodrive/local_map/test_cube";
        const std::string kSelf = "/autodrive/local_map/self";

        const std::string kLidarLeft = "/autodrive/local_map/lidar/left";
        const std::string kLidarRight = "/autodrive/local_map/lidar/right";
        const std::string kLidarCenter = "/autodrive/local_map/lidar/center";
        const std::string kImuTopic = "/autodrive/local_map/imu/imu";
        const std::string kImuAvgTopic = "/autodrive/local_map/imu/imu_avg";
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
        const std::string kCutoutPointcloud = "/autodrive/local_map/lidar/cutout";

        const std::string kLidarApproximation = "/autodrive/local_map/lidar/approximations";
        const std::string kLidarApproximationRoad = "/autodrive/local_map/lidar/approximations_road";

        const std::string kLidarDetections = "/autodrive/local_map/lidar_detections";

        const std::string kRadarTiObjects = "/autodrive/local_map/radar_ti_data";

        const std::string kTelemetryText = "/autodrive/local_map/telemetry/text";
        const std::string kSpeedTopic = "/autodrive/local_map/speed";

    }
}