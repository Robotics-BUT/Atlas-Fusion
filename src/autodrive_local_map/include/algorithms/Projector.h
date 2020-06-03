#pragma once
#include <opencv2/opencv.hpp>
#include <rtl/Vector3D.h>
#include <rtl/Vector2D.h>
#include <rtl/Transformation3D.h>

namespace AutoDrive::Algorithms {

    /**
     * Projector class holds the information about inner camera parameters and provides 3D to 2D and backward
     * projections w.r.t. the camera plain.
     */
    class Projector {

    public:

        /**
         * Constructor
         * @param intrinsic inner camera matrix
         * @param distortion camera's distortion coefs
         * @param tf reserved
         */
        Projector(const cv::Mat intrinsic, const cv::Mat distortion, rtl::Transformation3D<double>& tf)
        : intrinsic_{intrinsic}
        , distortion_{distortion}
        {

            auto rotMat = tf.rotQuaternion().rotMat();

            tvec_ = (cv::Mat_<float>(3, 1) << 0,0,0);
            rvec_ = (cv::Mat_<float>(3, 1) << 0,0,0);
        }

        /**
         * Projects points from 3D to the 2D
         * @param src input 3D points
         * @param dest output 2D points
         * @param useDist if true, distortion coefs will be used
         */
        void projectPoints(const std::vector<cv::Point3f>& src, std::vector<cv::Point2f>& dest, bool useDist = true);

        /**
         * Reverse projection from 2D to 3D. The output points represent 3D vector with the lenght of 1m.
         * @param src input 2D points
         * @param dest output 3D points
         * @param useDist if true, distortion coefs will be used
         */
        void reverseProjection(const std::vector<cv::Point2f>& src, std::vector<cv::Point3f>& dest, bool useDist = true);

        [[deprecated]] void getPointDirection(const std::vector<cv::Point2f>& src, std::vector<cv::Point3f>& dest);

        /**
         * Experimental code
         */
        void undistort(const std::vector<cv::Point2f>& src, std::vector<cv::Point2f>& dest);

        /**
         * Experimental code
         */
        void distort(const std::vector<cv::Point2f>& src, std::vector<cv::Point2f>& dest);

    private:

        const cv::Mat intrinsic_;
        const cv::Mat distortion_;
        cv::Mat rvec_;
        cv::Mat tvec_;

    };
}
