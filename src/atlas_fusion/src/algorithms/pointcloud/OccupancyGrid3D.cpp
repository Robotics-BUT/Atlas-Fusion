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

#include "algorithms/pointcloud/OccupancyGrid3D.h"

namespace AtlasFusion::Algorithms {

    cv::Mat OccupancyGrid3D::create_bird_view_from_data(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> data, const BirdViewParams& bird_view_params) const {

        size_t img_cols = (bird_view_params.left_margin() + bird_view_params.right_margin() ) / bird_view_params.cell_size();
        size_t img_rows = (bird_view_params.front_margin() + bird_view_params.back_margin() ) / bird_view_params.cell_size();
        cv::Mat output_img = cv::Mat::zeros(img_rows, img_cols, CV_8U);

        auto image_coords = point_clout_to_image_coords(data, bird_view_params);
        for (const auto& pt : *image_coords) {
            output_img.at<uint8_t>(pt.y(),pt.x()) = heightToColor(pt.z() * bird_view_params.cell_size(), -2, 3);
        }

        return output_img;
    }


    std::shared_ptr<std::vector<rtl::Vector3D<int>>> OccupancyGrid3D::point_clout_to_image_coords(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> data, const BirdViewParams& c) const {
        auto output = std::make_shared<std::vector<rtl::Vector3D<int>>>();

        size_t img_cols = (c.left_margin() + c.right_margin() ) / c.cell_size();
        size_t img_rows = (c.front_margin() + c.back_margin() ) / c.cell_size();


        for (const auto& pt : *data) {
            const auto p = pc_point_to_image_coords(pt, c);
            if (p.x() >= 0 && p.x() < img_cols && p.y() >= 0 && p.y() < img_rows) {
                output->push_back(p);
            }
        }
        return output;
    }


    rtl::Vector3D<int> OccupancyGrid3D::pc_point_to_image_coords(const pcl::PointXYZ& point, const BirdViewParams& c) const {

        size_t img_cols = (c.left_margin() + c.right_margin() ) / c.cell_size();
        size_t img_rows = (c.front_margin() + c.back_margin() ) / c.cell_size();

        return rtl::Vector3D<int>{static_cast<int>(std::round( -(1/c.cell_size()) * point.y + (img_cols-1.0) / 2.0)),
                                  static_cast<int>(std::round(-(1/c.cell_size()) * point.x + (img_rows-1.0)      )),
                                  static_cast<int>(std::round(point.z / c.cell_size()))};
    }



    uint8_t OccupancyGrid3D::heightToColor(float height, float minH, float maxH) const {
        auto dx = maxH - minH;
        auto dy = 255.0;
        auto k = dy / dx;
        auto q = -k * minH;
        auto val = k * height + q;
        return std::max(std::min(val, 255.0), 0.0);
    }
}