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

#include "data_writers/Lidar2ImagePlotter.h"
#include <algorithm>

#include "data_loader/RecordingConstants.h"

namespace AtlasFusion::DataWriters {


    std::shared_ptr<cv::Mat> Lidar2ImagePlotter::renderLidarPointsToImg(std::vector<cv::Point2f> points2D, std::vector<cv::Point3f> points3D, int imgWidth, int imgHeight, size_t pointSize) {

        auto image = std::make_shared<cv::Mat>(imgHeight, imgWidth, CV_8U, cv::Scalar(0));

        if(points2D.size() != points3D.size()) {
            context_.logger_.warning("points2D vector size is not equal to points3D vector size in Lidar2Image!");
        }

        for(size_t i = 0 ; i < points2D.size() ; i++) {
            image->at<uint8_t >(static_cast<int>(points2D.at(i).y), static_cast<int>(points2D.at(i).x)) = std::max(distanceToColor( pointLenght(points3D.at(i))),
                                                                                                                   image->at<uint8_t >(static_cast<int>(points2D.at(i).y),
                                                                                                                                       static_cast<int>(points2D.at(i).x)));

            if (pointSize > 1) {
                int padding = static_cast<int>(std::round((pointSize-1)/2));
                for (int j = -padding ; j <= padding; j ++) {
                    for (int k = -padding ; k <= padding; k ++) {
                        if (points2D.at(i).y + j < imgHeight && points2D.at(i).y + j >= 0 &&
                            points2D.at(i).x + k < imgWidth  && points2D.at(i).x + k >= 0 ) {
                            image->at<uint8_t>(static_cast<int>(points2D.at(i).y)+j,
                                               static_cast<int>(points2D.at(i).x)+k) = std::max(distanceToColor(pointLenght(points3D.at(i))),
                                                                                                                    image->at<uint8_t>(static_cast<int>(points2D.at(i).y)+j,
                                                                                                                                       static_cast<int>(points2D.at(i).x)+k));
                        }
                    }
                }
            }

        }

        return image;
    }

    uint8_t Lidar2ImagePlotter::distanceToColor(float dist) {

        auto val = (-255.0/maxDist_) * dist + 255.0;
        return static_cast<uint8_t> (std::max(std::min(val, 255.0), 0.0));
    }


    float Lidar2ImagePlotter::pointLenght(cv::Point3f& p) {
        return std::sqrt( p.x * p.x  +  p.y * p.y  +  p.z * p.z);
    }
}