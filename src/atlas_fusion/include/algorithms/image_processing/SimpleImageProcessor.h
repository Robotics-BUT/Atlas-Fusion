#pragma once

namespace AutoDrive::Algorithms {

    class SimpleImageProcessor {
    public:

        SimpleImageProcessor() = default;

        cv::Mat convertLeftFrontRGBToIrFieldOfView(cv::Mat) const;

    private:

    };
}
