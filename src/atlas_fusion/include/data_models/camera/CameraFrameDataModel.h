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
#include <iostream>

#include "data_models/GenericDataModel.h"
#include "data_models/all.h"

#include "data_models/DataModelTypes.h"
#include "data_loader/DataLoaderIdentifiers.h"

namespace AtlasFusion::DataModels {

    /**
     * Camera Frame Data Model represents single RGB frame captured by the camera
     */
    class CameraFrameDataModel : public GenericDataModel {

    public:

        /**
         * Constructor
         * @param timestamp frame capture timestamp
         * @param img image representation
         * @param innerTs inner camera timestamp
         * @param cameraIdentifier camera sensor identifier
         * @param yolo neural network's detections
         */
        CameraFrameDataModel(uint64_t timestamp, cv::Mat img, uint64_t innerTs, DataLoader::CameraIndentifier cameraIdentifier, std::vector<std::shared_ptr<YoloDetection>>& yolo)
        : GenericDataModel(timestamp)
        , image_(img)
        , innerCameraTimestamp_(innerTs)
        , yoloDetections_(std::move(yolo))
        , cameraIdentifier_(cameraIdentifier) {
            type_ = DataModelTypes::kCameraDataModelType;
        }

        std::string toString() override;

        /**
         * Image getter
         * @return opencv image
         */
        cv::Mat getImage() {return image_;};

        /**
         * inner camera getter
         * @return camera's timestam
         */
        uint64_t getInnerCameraTimestamp() {return innerCameraTimestamp_;};

        /**
         * Vector of NN's detections
         * @return NN's detections
         */
        std::vector<std::shared_ptr<YoloDetection>> getYoloDetections() {return yoloDetections_;};

        /**
         * NN's detections setter
         * @param detections
         */
        void setYoloDetections(std::vector<std::shared_ptr<YoloDetection>>& detections) {yoloDetections_ = detections;};

        /**
         * Unique camera sensor identifier getter
         * @return source sensor camera identifier
         */
        DataLoader::CameraIndentifier getCameraIdentifier() { return cameraIdentifier_;};

    private:

        cv::Mat image_;
        uint64_t innerCameraTimestamp_;
        std::vector<std::shared_ptr<YoloDetection>> yoloDetections_{} ;
        DataLoader::CameraIndentifier cameraIdentifier_;

    };
}