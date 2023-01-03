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
#include "../util/circularbuffer.h"

#define DFT_BLOCK_COUNT 8
#define DFT_WINDOW_SIZE 128
#define MAX_VISIBILITY_THRESHOLD 3400.0
#define MIN_VISIBILITY_THRESHOLD 2600.0
#define MOVING_AVERAGE_FORGET_RATE 0.005

#define HISTOGRAM_COUNT 12
#define HISTOGRAM_BINS 16
#define GLARE_THRESHOLD 0.6
#define OCCLUSION_THRESHOLD 0.0625
#define OCCLUSION_MIN_FRAMES 100

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
         * @param environmentalModel model of environment current state
         */
        CameraRGBFailChecker(Context &context, const Algorithms::SelfModel &selfModel, Algorithms::EnvironmentalModel &environmentalModel)
        : AbstractFailChecker{context, selfModel, environmentalModel} {
            for (int i = 0; i < HISTOGRAM_COUNT * HISTOGRAM_COUNT; i++) {
                occlusionBuffers.emplace_back(OCCLUSION_MIN_FRAMES);
            }
        }

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

        // Glare detection
        cv::Mat histograms = cv::Mat(HISTOGRAM_COUNT * HISTOGRAM_COUNT, HISTOGRAM_BINS, CV_32FC1, cv::Scalar(0.0f));
        cv::Mat glareAmounts = cv::Mat(HISTOGRAM_COUNT, HISTOGRAM_COUNT, CV_32FC1, cv::Scalar(0.0f));
        std::vector<CircularBuffer<bool>> occlusionBuffers;

        void estimateVanishingPoint();

        void calculateVisibility(bool isVanishingPoint, std::pair<int, int> centerPoint, std::pair<int, int> position);

        void detectGlareAndOcclusion();
    };
}

