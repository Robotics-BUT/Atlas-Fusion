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

    cv::Mat SimpleImageHandler::renderLidarPointsToImg(std::vector<cv::Point2f> points2D, std::vector<cv::Point3f> points3D, int imgWidth, int imgHeight, size_t pointSize, bool handle_height) {

        auto image = cv::Mat(imgHeight, imgWidth, CV_8U, cv::Scalar(0));

        if(points2D.size() != points3D.size()) {
            context_.logger_.warning("points2D vector size is not equal to points3D vector size in Lidar2Image!");
        }

        for(size_t i = 0 ; i < points2D.size() ; i++) {
            image.at<uint8_t >(static_cast<int>(points2D.at(i).y), static_cast<int>(points2D.at(i).x)) = std::max(distanceToColor( pointLenght(points3D.at(i))),
                                                                                                                  image.at<uint8_t >(static_cast<int>(points2D.at(i).y),
                                                                                                                                     static_cast<int>(points2D.at(i).x)));
            if (pointSize > 1) {
                int padding = static_cast<int>(std::round((pointSize-1)/2));
                for (int j = -padding ; j <= padding; j ++) {
                    for (int k = -padding ; k <= padding; k ++) {
                        if (points2D.at(i).y + j < imgHeight && points2D.at(i).y + j >= 0 &&
                            points2D.at(i).x + k < imgWidth  && points2D.at(i).x + k >= 0 ) {

                            if (handle_height) {
                                image.at<uint8_t>(static_cast<int>(points2D.at(i).y)+j,
                                                  static_cast<int>(points2D.at(i).x)+k) = std::max(distanceToColor(pointLenght(points3D.at(i))),
                                                                                                       image.at<uint8_t>(static_cast<int>(points2D.at(i).y)+j,
                                                                                                                         static_cast<int>(points2D.at(i).x)+k));
                            } else {
                                image.at<uint8_t>(static_cast<int>(points2D.at(i).y)+j,
                                                  static_cast<int>(points2D.at(i).x)+k) = std::max(distanceToColor(pointLenght(points3D.at(i)) - points3D.at(i).z),
                                                                                                   image.at<uint8_t>(static_cast<int>(points2D.at(i).y)+j,
                                                                                                                     static_cast<int>(points2D.at(i).x)+k));
                            }
                        }
                    }
                }
            }

        }
        return image;
    }



    cv::Mat SimpleImageHandler::renderSphericalPointsToImg(std::vector<SphericalPoint<float>> points, float thetaMargin, float phiMargin, float range, float angular_step, size_t pointSize) {
        size_t img_height = 2 * static_cast<size_t>(thetaMargin/angular_step);
        size_t img_width = 2 * static_cast<size_t>(phiMargin/angular_step);

        cv::Mat output = cv::Mat::zeros(img_height, img_width, CV_8U);

        for(size_t i = 0 ; i < points.size() ; i++) {
            int u = img_height/2 + rad2deg(points.at(i).theta() - M_PI_2) / angular_step;
            int v = img_width/2 - rad2deg(points.at(i).phi()) / angular_step;
            if (u >= 0 && u < img_height && v >= 0 && v < img_width) {
                output.at<uint8_t>(u, v) = std::max(distanceToColor(points.at(i).radius()), output.at<uint8_t>(u, v));
            }
            if (pointSize > 1) {
                int padding = static_cast<int>(std::round((pointSize-1)/2));
                for (int j = -padding ; j <= padding; j ++) {
                    for (int k = -padding ; k <= padding; k ++) {
                        if (u + j < img_height && u + j >= 0 &&
                            v + k < img_width  && v + k >= 0 ) {
                            output.at<uint8_t>(static_cast<int>(u)+j,
                                               static_cast<int>(v)+k) = std::max(distanceToColor(points.at(i).radius()),
                                                                                                     output.at<uint8_t>(static_cast<int>(u)+j,
                                                                                                                        static_cast<int>(v)+k));
                        }
                    }
                }
            }
        }

        return output;
    }



    uint8_t SimpleImageHandler::distanceToColor(float dist) {

        auto val = (-255.0/maxDist_) * dist + 255.0;
        return static_cast<uint8_t> (std::max(std::min(val, 255.0), 0.0));
    }


    float SimpleImageHandler::pointLenght(cv::Point3f& p) {
        return std::sqrt( p.x * p.x  +  p.y * p.y  +  p.z * p.z);
    }

    float SimpleImageHandler::deg2rad(float deg) {
        return deg / 180.0 * M_PI;
    }

    float SimpleImageHandler::rad2deg(float rad) {
        return rad / M_PI * 180.0;
    }
}