#pragma once


#include "data_loader/data_models/GenericDataModel.h"

#include <opencv2/opencv.hpp>
#include <iostream>

namespace AutoDrive {
    namespace DataLoader {

        class CameraIrFrameDataModel : public GenericDataModel {

        public:

            CameraIrFrameDataModel(uint64_t timestamp, cv::Mat img, double tempMin, double tempMax)
                    : GenericDataModel(timestamp)
                    , tempMin_(tempMin)
                    , tempMax_(tempMax){
                type_ = DataModelTypes::kCameraIrDataModelType;
                cv::cvtColor(img, image_, CV_BGR2GRAY);
            }

            std::string toString() override;
            cv::Mat getImage() {return image_;};
            std::pair<double, double> getTemp() {return std::pair<double, double>{tempMin_, tempMax_};};

        private:

            cv::Mat image_;
            double tempMin_;
            double tempMax_;
        };
    }
}
