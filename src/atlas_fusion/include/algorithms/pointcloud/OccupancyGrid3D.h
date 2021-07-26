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

#include "Context.h"
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <rtl/Core.h>

namespace AtlasFusion::Algorithms {

    /**
     * WORK IN PROGRESS
     */
    class OccupancyGrid3D {

    public:

        class BirdViewParams {
        public:
            BirdViewParams(float left_margin, float right_margin, float front_margin, float back_margin, float cell_size) :
                    left_margin_{left_margin},
                    right_margin_{right_margin},
                    front_margin_{front_margin},
                    back_margin_{back_margin},
                    cell_size_{cell_size} {}
            float left_margin() const {return left_margin_;}
            float right_margin() const {return right_margin_;}
            float front_margin() const {return front_margin_;}
            float back_margin() const {return back_margin_;}
            float cell_size() const {return cell_size_;}
        private:
            float left_margin_;
            float right_margin_;
            float front_margin_;
            float back_margin_;
            float cell_size_;
        };

        explicit OccupancyGrid3D(Context& context)
        : context_{context} {

        }

        [[nodiscard]] cv::Mat create_bird_view_from_data(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> data, const BirdViewParams& bird_view_params) const;

    private:

        std::shared_ptr<std::vector<rtl::Vector3D<int>>> point_clout_to_image_coords(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> data, const BirdViewParams& c) const;
        rtl::Vector3D<int> pc_point_to_image_coords(const pcl::PointXYZ& point, const BirdViewParams& bird_view_params) const;

        Context& context_;

        uint8_t heightToColor(float height, float minH, float maxH) const;
    };
}
