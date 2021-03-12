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
#include <opencv2/opencv.hpp>
#include <rtl/Core.h>
#include <rtl/Transformation.h>

namespace AtlasFusion::Algorithms {

    /**
     * Projector class holds the information about inner camera parameters and provides 3D to 2D and backward
     * projections w.r.t. the camera plain.
     */
    class Projector {

    public:

        /**
         * Constructor
         * @param intrinsic inner camera matrix
         * @param distortion camera's distortion coefs
         * @param tf reserved
         */
        Projector(const cv::Mat intrinsic, const cv::Mat distortion, rtl::RigidTf3D<double>& tf)
        : intrinsic_{intrinsic}
        , distortion_{distortion}
        {

            auto rotMat = tf.rotQuaternion().rotMat();

            tvec_ = (cv::Mat_<float>(3, 1) << 0,0,0);
            rvec_ = (cv::Mat_<float>(3, 1) << 0,0,0);
        }

        /**
         * Projects points from 3D to the 2D
         * @param src input 3D points
         * @param dest output 2D points
         * @param useDist if true, distortion coefs will be used
         */
        void projectPoints(const std::vector<cv::Point3f>& src, std::vector<cv::Point2f>& dest, bool useDist = true);

        /**
         * Reverse projection from 2D to 3D. The output points represent 3D vector with the lenght of 1m.
         * @param src input 2D points
         * @param dest output 3D points
         * @param useDist if true, distortion coefs will be used
         */
        void reverseProjection(const std::vector<cv::Point2f>& src, std::vector<cv::Point3f>& dest, bool useDist = true);

        [[deprecated]] void getPointDirection(const std::vector<cv::Point2f>& src, std::vector<cv::Point3f>& dest);

        /**
         * Experimental code
         */
        void undistort(const std::vector<cv::Point2f>& src, std::vector<cv::Point2f>& dest);

        /**
         * Experimental code
         */
        void distort(const std::vector<cv::Point2f>& src, std::vector<cv::Point2f>& dest);

    private:

        const cv::Mat intrinsic_;
        const cv::Mat distortion_;
        cv::Mat rvec_;
        cv::Mat tvec_;

    };
}
