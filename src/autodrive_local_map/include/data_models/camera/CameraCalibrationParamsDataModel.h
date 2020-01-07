#pragma once

#include <opencv2/opencv.hpp>

namespace AutoDrive::DataModels {

    class CameraCalibrationParamsDataModel : public GenericDataModel {

    public:

        CameraCalibrationParamsDataModel(size_t width, size_t height, std::vector<std::vector<double>>& intrinsic, std::vector<double>& distortion)
        : GenericDataModel(0)
        , width_{width}
        , height_{height}
        , intrinsic_ {intrinsic}
        , distortion_ {distortion} {
            type_ = DataModelTypes::kCameraCalibrationParamsDataModelType;
        }

        size_t getWidth() {return width_;};
        size_t getHeight() {return height_;};
        const std::vector<std::vector<double>> getIntrinsicParams() {return intrinsic_;};
        const std::vector<double> getDistortionParams() {return distortion_;};

        const cv::Mat getMatIntrinsicParams() {
            cv::Mat mat = (cv::Mat_<double>(3, 3) <<
                    intrinsic_[0][0], intrinsic_[0][1], intrinsic_[0][2],
                    intrinsic_[1][0], intrinsic_[1][1], intrinsic_[1][2],
                    intrinsic_[2][0], intrinsic_[2][1], intrinsic_[2][2]);
            return mat;
        };

        const cv::Mat getMatDistortionParams() {
            cv::Mat mat = (cv::Mat_<double>(1, 5) <<
                   distortion_[0], distortion_[1], distortion_[2], distortion_[3], distortion_[4]);
            return mat;
        };

    private:

        size_t width_;
        size_t height_;
        std::vector<std::vector<double>> intrinsic_;
        std::vector<double> distortion_;
    };

}