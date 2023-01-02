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

#include "AbstractFailChecker.h"
#include "data_models/camera/CameraFrameDataModel.h"

#define DFT_BLOCK_COUNT 8
#define DFT_WINDOW_SIZE 128
#define MAX_VISIBILITY_THRESHOLD 3400.0
#define MIN_VISIBILITY_THRESHOLD 2600.0
#define MOVING_AVERAGE_FORGET_RATE 0.005

namespace AutoDrive::FailCheck {

    /**
     * Validates RGB camera frame data. Currently bypassed.
     */
    class CameraRGBFailChecker : public AbstractFailChecker {

    public:

        CameraRGBFailChecker() = delete;

        /**
         * Constructor
         * @param context container for global services (timestamps. logging, etc.)
         * @param selfModel self model of ego vehicle
         */
        CameraRGBFailChecker(Context &context, const Algorithms::SelfModel& selfModel) : AbstractFailChecker{context, selfModel} {}

        /**
         * Pipe to provide new sensor data into the Camera RGB Fail Checker
         * @param data RGB camera data frame
         */
        void onNewData(const std::shared_ptr<DataModels::CameraFrameDataModel>& data);

    private:

        cv::Mat frameBgr{};
        cv::Mat frameGray{};

        double vanishingPointX = 0.0;
        double vanishingPointY = 0.0;

        double vanishingPointVisibility = 1.0;
        cv::Mat visibility = cv::Mat(DFT_BLOCK_COUNT, DFT_BLOCK_COUNT, CV_32FC1, cv::Scalar(0.0f));
        void estimateVanishingPoint();

        void calculateVisibility(bool isVanishingPoint, std::pair<int, int> centerPoint, std::pair<int, int> position);
    };
}

