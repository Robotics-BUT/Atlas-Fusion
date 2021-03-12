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
    class DepthMapDataModel : public GenericDataModel {

    public:

        /**
         * Constructor
         * @param timestamp frame capture timestamp
         * @param img image representation
         * @param cameraIdentifier camera sensor identifier
         * @param yolo neural network's detections
         */
        DepthMapDataModel(uint64_t timestamp, cv::Mat img, DataLoader::CameraIndentifier cameraIdentifier, const std::vector<std::shared_ptr<YoloDetection>>& yolo)
                : GenericDataModel(timestamp)
                , image_(img)
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
        std::vector<std::shared_ptr<YoloDetection>> yoloDetections_{} ;
        DataLoader::CameraIndentifier cameraIdentifier_;

    };
}