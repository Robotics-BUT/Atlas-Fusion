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

#include "algorithms/Projector.h"

namespace AtlasFusion::Algorithms {


    void Projector::projectPoints(const std::vector<cv::Point3f>& src, std::vector<cv::Point2f>& dest, bool useDist) {

        std::vector<cv::Point2f> tmp;
        if(!src.empty()) {
            if(useDist) {
                cv::projectPoints(src, rvec_, tvec_, intrinsic_, distortion_, dest);
            } else {
                cv::projectPoints(src, rvec_, tvec_, intrinsic_, {}, dest);
            }
        }
    }



    void Projector::reverseProjection(const std::vector<cv::Point2f>& src, std::vector<cv::Point3f>& dest, bool useDist) {

        std::vector<cv::Point2f> undistorted;
        if (useDist) {
            cv::undistortPoints(src, undistorted, intrinsic_, distortion_);
        } else {
            cv::undistortPoints(src, undistorted, intrinsic_, {});
        }

        for(const auto& p : undistorted) {
            float denominatr = static_cast<float>(std::sqrt( std::pow(p.x,2) + std::pow(p.y,2) + 1 ));
            cv::Point3f direction = {p.x/denominatr, p.y/denominatr, 1/denominatr};
            dest.push_back(direction);
        }
    }


    void Projector::getPointDirection(const std::vector<cv::Point2f>& src, std::vector<cv::Point3f>& dest) {

        std::vector<cv::Point2f> undistorted;
        cv::undistortPoints(src, undistorted, intrinsic_, cv::Mat());

        for(const auto& p : undistorted) {
            float denominatr = static_cast<float>(std::sqrt( std::pow(p.x,2) + std::pow(p.y,2) + 1 ));
            cv::Point3f direction = {p.x/denominatr, p.y/denominatr, 1/denominatr};
            dest.push_back(direction);
        }
    }


    void Projector::undistort(const std::vector<cv::Point2f>& src, std::vector<cv::Point2f>& dest) {

        auto fx = intrinsic_.at<double>(0,0);
        auto fy = intrinsic_.at<double>(1,1);
        auto cx = intrinsic_.at<double>(0,2);
        auto cy = intrinsic_.at<double>(1,2);

        cv::undistortPoints(src, dest, intrinsic_, distortion_);

        for(auto& p : dest) {
            p.x = (p.x * fx + cx);
            p.y = (p.y * fy + cy);
        }
    }


    void Projector::distort(const std::vector<cv::Point2f>& src, std::vector<cv::Point2f>& dest) {

        double k1 = distortion_.at<double>(0);
        double k2 = distortion_.at<double>(1);
        double p1 = distortion_.at<double>(2);
        double p2 = distortion_.at<double>(3);
        double k3 = distortion_.at<double>(4);

        auto fx = intrinsic_.at<double>(0,0);
        auto fy = intrinsic_.at<double>(1,1);

        auto cx = intrinsic_.at<double>(0,2);
        auto cy = intrinsic_.at<double>(1,2);

        for(const auto& point : src) {
            double x = (point.x - cx) / fx;
            double y = (point.y - cy) / fy;

            double r2 =  x * x + y * y;

            // Radial distorsion
            double xDistort = x * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);
            double yDistort = y * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);

            // Tangential distorsion
            xDistort = xDistort + (2 * p1 * x * y + p2 * (r2 + 2 * x * x));
            yDistort = yDistort + (p1 * (r2 + 2 * y * y) + 2 * p2 * x * y);

            // Back to absolute coordinates.
            xDistort = xDistort * fx + cx;
            yDistort = yDistort * fy + cy;

            dest.push_back(cv::Point2f{static_cast<float>(xDistort),
                                       static_cast<float>(yDistort)});
        }
    }
}