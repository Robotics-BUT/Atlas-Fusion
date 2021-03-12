/*
 * Copyright 2021 Brno University of Technology
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

namespace AtlasFusion {


    class FunctionalityFlags {

    public:

        FunctionalityFlags() = default;

        FunctionalityFlags(bool generate_depth_map_for_ir,
                           bool rgb_to_ir_detection_projection,
                           bool short_term_lidar_aggregation,
                           bool lidar_laser_approximations_and_segmentation,
                           bool global_lidar_aggregation,
                           bool visualization_global_enable,
                           bool rgb_camera_visualization,
                           bool ir_camera_visualization,
                           bool lidar_visualization,
                           bool imu_visualization,
                           bool gnss_visualization,
                           bool radar_visualization) :
                generate_depth_map_for_ir_{generate_depth_map_for_ir},
                rgb_to_ir_detection_projection_{rgb_to_ir_detection_projection},
                short_term_lidar_aggregation_{short_term_lidar_aggregation},
                lidar_laser_approximations_and_segmentation_{lidar_laser_approximations_and_segmentation},
                global_lidar_aggregation_{global_lidar_aggregation},
                visualization_global_enable_{visualization_global_enable},
                rgb_camera_visualization_{rgb_camera_visualization},
                ir_camera_visualization_{ir_camera_visualization},
                lidar_visualization_{lidar_visualization},
                imu_visualization_{imu_visualization},
                gnss_visualization_{gnss_visualization},
                radar_visualization_{radar_visualization} {

        }

        const bool generate_depth_map_for_ir_ = false;
        const bool rgb_to_ir_detection_projection_ = false;
        const bool short_term_lidar_aggregation_ = false;
        const bool lidar_laser_approximations_and_segmentation_ = false;
        const bool global_lidar_aggregation_ = false;
        const bool visualization_global_enable_ = false;
        const bool rgb_camera_visualization_ = false;
        const bool ir_camera_visualization_ = false;
        const bool lidar_visualization_ = false;
        const bool imu_visualization_ = false;
        const bool gnss_visualization_ = false;
        const bool radar_visualization_ = false;
    };
};