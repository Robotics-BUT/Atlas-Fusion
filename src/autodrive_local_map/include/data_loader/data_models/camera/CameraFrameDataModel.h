#pragma once

#include "data_loader/data_models/GenericDataModel.h"
#include "data_loader/data_models/all.h"
#include <opencv2/opencv.hpp>
#include <iostream>

namespace AutoDrive {
    namespace DataLoader {


        class CameraFrameDataModel : public GenericDataModel {

        public:

            CameraFrameDataModel(uint64_t timestamp, cv::Mat img, uint64_t innerTs, std::vector<std::shared_ptr<YoloDetection>> yolo)
            : GenericDataModel(timestamp)
            , image_(img)
            , innerCameraTimestamp_(innerTs)
            , yoloDetections_(yolo) {
                type_ = DataModelTypes::kCameraDataModelType;
            }

            std::string toString() override;
            cv::Mat getImage() {return image_;};
            uint64_t getInnerCameraTimestamp() {return innerCameraTimestamp_;};
            std::vector<std::shared_ptr<YoloDetection>> getYoloDetections() {return yoloDetections_;};
            void setYoloDetections(std::vector<std::shared_ptr<YoloDetection>>& detections) {yoloDetections_ = detections;};

        private:

            cv::Mat image_;
            uint64_t innerCameraTimestamp_;
            std::vector<std::shared_ptr<YoloDetection>> yoloDetections_{} ;

        };
    }
}