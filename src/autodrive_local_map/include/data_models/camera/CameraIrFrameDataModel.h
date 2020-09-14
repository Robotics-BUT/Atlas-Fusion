#pragma once

#include "data_models/GenericDataModel.h"

#include <opencv2/opencv.hpp>
#include <iostream>

#include "data_loader/DataLoaderIdentifiers.h"

namespace AutoDrive::DataModels {

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
        CameraIrFrameDataModel(uint64_t timestamp, cv::Mat img, double tempMin, double tempMax, DataLoader::CameraIndentifier cameraIdentifier)
                : GenericDataModel(timestamp)
                , tempMin_(tempMin)
                , tempMax_(tempMax)
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
         * Camera sensor identifier
         * @return unique camera sensor identifier
         */
        DataLoader::CameraIndentifier getCameraIdentifier() { return cameraIdentifier_;};

    private:

        cv::Mat image_;
        double tempMin_;
        double tempMax_;
        DataLoader::CameraIndentifier cameraIdentifier_;
    };
}
