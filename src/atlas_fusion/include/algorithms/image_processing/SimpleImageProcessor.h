#pragma once

#include <opencv2/opencv.hpp>

namespace AutoDrive::Algorithms {

    class SimpleImageProcessor {
    public:

        SimpleImageProcessor() = default;

        cv::Mat convertLeftFrontRGBToIrFieldOfView(cv::Mat) const;

    private:

    };
}
