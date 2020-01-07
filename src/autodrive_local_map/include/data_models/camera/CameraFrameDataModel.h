#pragma once

#include <opencv2/opencv.hpp>
#include <iostream>

#include "data_models/GenericDataModel.h"
#include "data_models/all.h"

#include "data_models/DataModelTypes.h"
#include "data_loader/DataLoaderIdentifiers.h"

namespace AutoDrive::DataModels {

    class CameraFrameDataModel : public GenericDataModel {

    public:

        CameraFrameDataModel(uint64_t timestamp, cv::Mat img, uint64_t innerTs, DataLoader::CameraIndentifier cameraIdentifier, std::vector<std::shared_ptr<YoloDetection>>& yolo)
        : GenericDataModel(timestamp)
        , image_(img)
        , innerCameraTimestamp_(innerTs)
        , yoloDetections_(std::move(yolo))
        , cameraIdentifier_(cameraIdentifier) {
            type_ = DataModelTypes::kCameraDataModelType;
        }

        std::string toString() override;
        cv::Mat getImage() {return image_;};
        uint64_t getInnerCameraTimestamp() {return innerCameraTimestamp_;};
        std::vector<std::shared_ptr<YoloDetection>> getYoloDetections() {return yoloDetections_;};
        void setYoloDetections(std::vector<std::shared_ptr<YoloDetection>>& detections) {yoloDetections_ = detections;};
        DataLoader::CameraIndentifier getCameraIdentifier() { return cameraIdentifier_;};

    private:

        cv::Mat image_;
        uint64_t innerCameraTimestamp_;
        std::vector<std::shared_ptr<YoloDetection>> yoloDetections_{} ;
        DataLoader::CameraIndentifier cameraIdentifier_;

    };
}