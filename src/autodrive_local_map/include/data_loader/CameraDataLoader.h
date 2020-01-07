#pragma once

#include <iostream>
#include <memory>
#include <vector>
#include <opencv2/opencv.hpp>

#include "data_models/all.h"
#include "AbstractDataLoader.h"
#include "data_models/DataModelTypes.h"
#include "Context.h"

namespace AutoDrive {
    namespace DataLoader {

        class CameraDataLoader : public AbstractDataLoader {

            struct CameraFrame {

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

            CameraIndentifier getCameraIdentifier() {return cameraIdentifier_;};
            std::shared_ptr<DataModels::CameraCalibrationParamsDataModel> getCameraCalibParams();

        private:

            Context& context_;
            CameraIndentifier cameraIdentifier_;
            std::string cameraCalibFilePath_;
            cv::VideoCapture video_;
            std::vector<std::shared_ptr<CameraFrame>> data_;
            std::vector<std::shared_ptr<CameraFrame>>::iterator dataIt_;
            std::vector<std::shared_ptr<CameraFrame>>::iterator releaseIt_;

            void loadYoloDetections(const std::string& path) const;
            void loadCameraCalibParams();
        };
    }
}
