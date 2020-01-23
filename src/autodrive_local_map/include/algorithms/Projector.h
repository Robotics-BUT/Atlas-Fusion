#pragma once
#include <opencv2/opencv.hpp>
#include <rtl/Vector3D.h>
#include <rtl/Vector2D.h>
#include <rtl/Transformation3D.h>

namespace AutoDrive::Algorithms {

    class Projector {

    public:

        Projector(const cv::Mat intrinsic, const cv::Mat distortion, rtl::Transformation3D<double>& tf)
        : intrinsic_{intrinsic}
        , distortion_{distortion}
        {

            auto rotMat = tf.rotQuaternion().rotMat();
//            cv::Mat rot = (cv::Mat_<float>(3, 3) <<
//                    rotMat(0, 0), rotMat(1, 1), rotMat(2, 0),
//                    rotMat(0, 1), rotMat(1, 2), rotMat(2, 1),
//                    rotMat(0, 2), rotMat(1, 2), rotMat(2, 2));

            tvec_ = (cv::Mat_<float>(3, 1) << 0,0,0);
            rvec_ = (cv::Mat_<float>(3, 1) << 0,0,0);
//            cv::Rodrigues(rot, rvec_);
        }

        void projectPoints(const std::vector<cv::Point3f>& src, std::vector<cv::Point2f>& dest);
        void reverseProjection(const std::vector<cv::Point2f>& src, std::vector<cv::Point3f>& dest);

        void getPointDirection(const std::vector<cv::Point2f>& src, std::vector<cv::Point3f>& dest);

        void undistort(const std::vector<cv::Point2f>& src, std::vector<cv::Point2f>& dest);
        void distort(const std::vector<cv::Point2f>& src, std::vector<cv::Point2f>& dest);

    private:

        const cv::Mat intrinsic_;
        const cv::Mat distortion_;
        cv::Mat rvec_;
        cv::Mat tvec_;

    };
}
