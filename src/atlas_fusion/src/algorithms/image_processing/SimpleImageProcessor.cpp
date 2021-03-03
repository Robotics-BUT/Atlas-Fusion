#include "algorithms/image_processing/SimpleImageProcessor.h"

namespace AutoDrive::Algorithms {

    cv::Mat SimpleImageProcessor::convertLeftFrontRGBToIrFieldOfView(cv::Mat rgb) const {

        cv::Rect cutout( 90, 0, 1736, 1200 );
        cv::Mat rgb_cutout = rgb(cutout);
//        cv::imshow("cutout", rgb_cutout);
//        cv::waitKey(0);

        cv::Mat big_screen = cv::Mat::zeros(1374, 1736, CV_8UC3);
        rgb_cutout.copyTo(big_screen(cv::Rect_<int>(0,82,1736,1200)));
//        cv::imshow("big_screen", big_screen);
//        cv::waitKey(0);

        cv::Mat resized;
        cv::resize(big_screen, resized, cv::Size(640,512));
//        cv::imshow("resized", resized);
//        cv::waitKey(0);

        return resized;
    }
}