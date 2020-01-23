#include "algorithms/Projector.h"

namespace AutoDrive::Algorithms {


    void Projector::projectPoints(const std::vector<cv::Point3f>& src, std::vector<cv::Point2f>& dest) {

        std::vector<cv::Point2f> tmp;
        if(!src.empty()) {
            cv::projectPoints(src, rvec_, tvec_, intrinsic_, distortion_, dest);
        }
    }



    void Projector::reverseProjection(const std::vector<cv::Point2f>& src, std::vector<cv::Point3f>& dest) {

        std::vector<cv::Point2f> undistorted;
        cv::undistortPoints(src, undistorted, intrinsic_, distortion_);

        for(const auto& p : undistorted) {
            float denominatr = static_cast<float>(std::sqrt( std::pow(p.x,2) + std::pow(p.y,2) + 1 ));
            cv::Point3f direction = {p.x/denominatr, p.y/denominatr, 1/denominatr};
            dest.push_back(direction);
        }
    }


    void Projector::getPointDirection(const std::vector<cv::Point2f>& src, std::vector<cv::Point3f>& dest) {

        std::vector<cv::Point2f> undistorted;
        cv::undistortPoints(src, undistorted, intrinsic_, cv::Mat());

        for(const auto& p : undistorted) {
            float denominatr = static_cast<float>(std::sqrt( std::pow(p.x,2) + std::pow(p.y,2) + 1 ));
            cv::Point3f direction = {p.x/denominatr, p.y/denominatr, 1/denominatr};
            dest.push_back(direction);
        }
    }


    void Projector::undistort(const std::vector<cv::Point2f>& src, std::vector<cv::Point2f>& dest) {

        auto fx = intrinsic_.at<double>(0,0);
        auto fy = intrinsic_.at<double>(1,1);
        auto cx = intrinsic_.at<double>(0,2);
        auto cy = intrinsic_.at<double>(1,2);

        cv::undistortPoints(src, dest, intrinsic_, distortion_);

        for(auto& p : dest) {
            p.x = (p.x * fx + cx);
            p.y = (p.y * fy + cy);
        }
    }


    void Projector::distort(const std::vector<cv::Point2f>& src, std::vector<cv::Point2f>& dest) {

        double k1 = distortion_.at<double>(0);
        double k2 = distortion_.at<double>(1);
        double p1 = distortion_.at<double>(2);
        double p2 = distortion_.at<double>(3);
        double k3 = distortion_.at<double>(4);

        auto fx = intrinsic_.at<double>(0,0);
        auto fy = intrinsic_.at<double>(1,1);

        auto cx = intrinsic_.at<double>(0,2);
        auto cy = intrinsic_.at<double>(1,2);

        for(const auto& point : src) {
            double x = (point.x - cx) / fx;
            double y = (point.y - cy) / fy;

            double r2 =  x * x + y * y;

            // Radial distorsion
            double xDistort = x * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);
            double yDistort = y * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);

            // Tangential distorsion
            xDistort = xDistort + (2 * p1 * x * y + p2 * (r2 + 2 * x * x));
            yDistort = yDistort + (p1 * (r2 + 2 * y * y) + 2 * p2 * x * y);

            // Back to absolute coordinates.
            xDistort = xDistort * fx + cx;
            yDistort = yDistort * fy + cy;

            dest.push_back(cv::Point2f{static_cast<float>(xDistort),
                                       static_cast<float>(yDistort)});
        }
    }
}