#pragma once

#include <iostream>
#include <memory>
#include <vector>
#include <opencv2/opencv.hpp>

#include "data_models/all.h"
#include "AbstractDataLoader.h"
#include "data_models/DataModelTypes.h"

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
                std::vector<std::shared_ptr<YoloDetection>> detections_{};
            };

        public:

            CameraDataLoader(CameraIndentifier id)
            : cameraIdentifier_(id){
                data_.clear();
                dataIt_ = data_.begin();
            }

            bool loadData(std::string path) override;
            timestamp_type getLowestTimestamp() override;
            std::shared_ptr<GenericDataModel> getNextData() override;
            std::string toString() override;
            uint64_t getDataSize() override;
            bool isOnEnd() override;
            void rewind() override;
            void setPose(timestamp_type) override;
            void clear() override;


        private:

            CameraIndentifier cameraIdentifier_;
            cv::VideoCapture video_;
            std::vector<std::shared_ptr<CameraFrame>> data_;
            std::vector<std::shared_ptr<CameraFrame>>::iterator dataIt_;

            void loadYoloDetections(const std::string& path) const;
        };
    }
}
