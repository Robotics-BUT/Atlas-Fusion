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

#include <iostream>
#include <memory>
#include <vector>
#include <opencv2/opencv.hpp>

#include "data_models/all.h"
#include "AbstractDataLoader.h"
#include "data_models/DataModelTypes.h"
#include "Context.h"

namespace AtlasFusion {
    namespace DataLoader {


        /**
         * Camera Data Loader handles video files lazy loading, calibration parameters loading and the neural network's
         * detections and provides these data on request to the upper structures.
         */
        class CameraDataLoader : public AbstractDataLoader {

            /**
             * Simple structure that represents the frames in the video file, before the video frame is loaded.
             */
            struct CameraFrame {

                /**
                 * Constructor
                 * @param ts Recording timestamp
                 * @param fId frame position in the video sequence
                 * @param iTs inner camera's timestamp
                 * @param tempMin minimal image temp (only for IR frames, for RGB not valid)
                 * @param tempMax maximal image temp (only for IR frames, for RGB not valid)
                 */
                CameraFrame(uint64_t ts, uint32_t fId, uint64_t iTs, double tempMin, double tempMax)
                : timestamp_(ts)
                , frameId_(fId)
                , innerTimestamp_(iTs)
                , tempMin_(tempMin)
                , tempMax_(tempMax) {}

                timestamp_type getTimestamp() {return timestamp_;};

                timestamp_type timestamp_;
                uint32_t frameId_;
                uint64_t innerTimestamp_;
                double tempMin_;
                double tempMax_;
                std::vector<std::shared_ptr<DataModels::YoloDetection>> detections_{};
            };

        public:

            /**
             * Constructor
             * @param context global services container (timestamps, logging, etc.)
             * @param id unique identifier of the data loader. Related to the sensor.
             * @param calibFilePath path to the camera calibration file
             */
            CameraDataLoader(Context& context, CameraIndentifier id, std::string calibFilePath)
            : context_{context}
            , cameraIdentifier_(id)
            , cameraCalibFilePath_{calibFilePath} {
                data_.clear();
                dataIt_ = data_.begin();
            }

            bool loadData(std::string path) override;
            timestamp_type getLowestTimestamp() override;
            std::shared_ptr<DataModels::GenericDataModel> getNextData() override;
            std::string toString() override;
            uint64_t getDataSize() override;
            bool isOnEnd() override;
            void setPose(timestamp_type) override;
            void releaseOldData(timestamp_type keepHistory) override;
            void clear() override;

            /**
             * Method returns the unique identifier, that identifies Data Loader with specific sensor
             * @return Data Loader identifier
             */
            CameraIndentifier getCameraIdentifier() {return cameraIdentifier_;};

            /**
             * Method returns the calibration parameters for a camera that the Data Loader is responsible for.
             * @return camera's calibration parameters.
             */
            std::shared_ptr<DataModels::CameraCalibrationParamsDataModel> getCameraCalibParams();

        private:

            Context& context_;
            CameraIndentifier cameraIdentifier_;
            std::string cameraCalibFilePath_;
            cv::VideoCapture video_;
            std::vector<std::shared_ptr<CameraFrame>> data_;
            std::vector<std::shared_ptr<CameraFrame>>::iterator dataIt_;
            std::vector<std::shared_ptr<CameraFrame>>::iterator releaseIt_;
            std::pair<int, int> imageWidthHeight_ = {-1, -1};

            void loadYoloDetections(const std::string& path) const;
            void loadCameraCalibParams();
        };
    }
}
