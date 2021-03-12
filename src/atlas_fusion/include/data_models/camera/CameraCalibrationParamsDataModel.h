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

namespace AtlasFusion::DataModels {

    /**
     * Camera Calibration Parameters Data Model keeps information about the image dimensions, camera calibration matrix
     * and the distortion coeficients
     */
    class CameraCalibrationParamsDataModel : public GenericDataModel {

    public:

        /**
         * Constructor
         * @param width image width
         * @param height imahge height
         * @param intrinsic intrinsic camera parameters
         * @param distortion image distortion coeficients
         */
        CameraCalibrationParamsDataModel(size_t width, size_t height, std::vector<std::vector<double>>& intrinsic, std::vector<double>& distortion)
        : GenericDataModel(0)
        , width_{width}
        , height_{height}
        , intrinsic_ {intrinsic}
        , distortion_ {distortion} {
            type_ = DataModelTypes::kCameraCalibrationParamsDataModelType;
        }

        /**
         * Image width getter
         * @return image width
         */
        size_t getWidth() {return width_;};

        /**
         * Image height getter
         * @return image height
         */
        size_t getHeight() {return height_;};

        /**
         * Intrinsic camera parameters getter
         * @return matrix 3x3 represented as a vector of vectors
         */
        const std::vector<std::vector<double>> getIntrinsicParams() {return intrinsic_;};

        /**
         * Camera distortion coefficients
         * @return distortion coefficients as a vector
         */
        const std::vector<double> getDistortionParams() {return distortion_;};

        /**
         * Intrinsic camera parameters getter
         * @return matrix 3x3 represented as a opencv mat
         */
        const cv::Mat getMatIntrinsicParams() {
            cv::Mat mat = (cv::Mat_<double>(3, 3) <<
                    intrinsic_[0][0], intrinsic_[0][1], intrinsic_[0][2],
                    intrinsic_[1][0], intrinsic_[1][1], intrinsic_[1][2],
                    intrinsic_[2][0], intrinsic_[2][1], intrinsic_[2][2]);
            return mat;
        };

        /**
         * Camera distortion coefficients
         * @return distortion coefficients as a opencv mat
         */
        const cv::Mat getMatDistortionParams() {
            cv::Mat mat = (cv::Mat_<double>(1, 5) <<
                   distortion_[0], distortion_[1], distortion_[2], distortion_[3], distortion_[4]);
            return mat;
        };

    private:

        size_t width_;
        size_t height_;
        std::vector<std::vector<double>> intrinsic_;
        std::vector<double> distortion_;
    };

}