#include "algorithms/image_processing/SimpleImageHandler.h"

namespace AtlasFusion::Algorithms {

    cv::Mat SimpleImageHandler::convertLeftFrontRGBToIrFieldOfView(cv::Mat rgb) const {

        cv::Rect cutout( 90, 0, 1736, 1200 );
        cv::Mat rgb_cutout = rgb(cutout);


        cv::Mat big_screen = cv::Mat::zeros(1374, 1736, CV_8UC3);
        rgb_cutout.copyTo(big_screen(cv::Rect_<int>(0,82,1736,1200)));

        cv::Mat resized;
        cv::resize(big_screen, resized, cv::Size(640,512));

        return resized;
    }

    cv::Mat SimpleImageHandler::renderLidarPointsToImg(std::vector<cv::Point2f> points2D, std::vector<cv::Point3f> points3D, int imgWidth, int imgHeight, size_t pointSize) {

        auto image = cv::Mat(imgHeight, imgWidth, CV_8U, cv::Scalar(0));

        if(points2D.size() != points3D.size()) {
            context_.logger_.warning("points2D vector size is not equal to points3D vector size in Lidar2Image!");
        }

        for(size_t i = 0 ; i < points2D.size() ; i++) {
            image.at<uint8_t >(static_cast<int>(points2D.at(i).y), static_cast<int>(points2D.at(i).x)) = distanceToColor( pointLenght(points3D.at(i)) );
            if (pointSize > 1) {
                int padding = static_cast<int>(std::round((pointSize-1)/2));
                for (int j = -padding ; j <= padding; j ++) {
                    for (int k = -padding ; k <= padding; k ++) {
                        if (points2D.at(i).y + j < imgHeight && points2D.at(i).y + j >= 0 &&
                            points2D.at(i).x + k < imgWidth  && points2D.at(i).x + k >= 0 ) {
                            image.at<uint8_t>(static_cast<int>(points2D.at(i).y)+j,
                                               static_cast<int>(points2D.at(i).x)+k) = distanceToColor(pointLenght(points3D.at(i)));
                        }
                    }
                }
            }

        }
        return image;
    }

    uint8_t SimpleImageHandler::distanceToColor(float dist) {

        auto val = (-255.0/maxDist_) * dist + 255.0;
        return static_cast<uint8_t> (std::max(std::min(val, 255.0), 0.0));
    }


    float SimpleImageHandler::pointLenght(cv::Point3f& p) {
        return std::sqrt( p.x * p.x  +  p.y * p.y  +  p.z * p.z);
    }
}