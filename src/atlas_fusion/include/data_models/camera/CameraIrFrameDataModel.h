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

#include "data_models/GenericDataModel.h"

#include <opencv2/opencv.hpp>
#include <iostream>

#include "data_models/all.h"
#include "data_loader/DataLoaderIdentifiers.h"

namespace AtlasFusion::DataModels {

    /**
     * Camera IR Frame Data Model represents the thermal image captured by the IR camera. Contains timestamp, image
     * itself and the temperature range
     */
    class CameraIrFrameDataModel : public GenericDataModel {

    public:

        /**
         * Constructor
         * @param timestamp image capture timestamp
         * @param img image data
         * @param tempMin maximal temperature measured on image
         * @param tempMax minimal temperature measured on image
         * @param cameraIdentifier unique camera identifier
         */
        CameraIrFrameDataModel(uint64_t timestamp, cv::Mat img, double tempMin, double tempMax, const DataLoader::CameraIndentifier& cameraIdentifier, const std::vector<std::shared_ptr<YoloDetection>>& yolo)
                : GenericDataModel(timestamp)
                , tempMin_(tempMin)
                , tempMax_(tempMax)
                , yoloDetections_(yolo)
                , cameraIdentifier_{cameraIdentifier} {
            type_ = DataModelTypes::kCameraIrDataModelType;
            cv::cvtColor(img, image_, cv::COLOR_BGR2GRAY);
        }

        std::string toString() override;

        /**
         * IR Image getter
         * @return thermal image (grayscale)
         */
        cv::Mat getImage() {return image_;};

        /**
         * Minimal and maximal temperatures captured on the thermal image
         * @return min and max temperatures on the image
         */
        std::pair<double, double> getTemp() {return std::pair<double, double>{tempMin_, tempMax_};};

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
         * Camera sensor identifier
         * @return unique camera sensor identifier
         */
        DataLoader::CameraIndentifier getCameraIdentifier() { return cameraIdentifier_;};

    private:

        cv::Mat image_;
        double tempMin_;
        double tempMax_;
        std::vector<std::shared_ptr<YoloDetection>> yoloDetections_{} ;
        DataLoader::CameraIndentifier cameraIdentifier_;
    };
}
