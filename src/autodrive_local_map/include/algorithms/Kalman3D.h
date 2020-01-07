#pragma once

#include <opencv2/video/tracking.hpp>
#include <iostream>

namespace AutoDrive::Algorithms {

    class Kalman3D {

    public:

        Kalman3D(double processNoise, double observationNoise)
                : processNoise_(processNoise)
                , observationNoise_(observationNoise) {
            states_ = (cv::Mat_<double>(6, 1) <<
                    0,
                    0,
                    0,
                    0,
                    0,
                    0);
            covariance_ = (cv::Mat_<double>(6, 6) <<
                    1, 0, 0, 0, 0, 0,
                    0, 1, 0, 0, 0, 0,
                    0, 0, 1, 0, 0, 0,
                    0, 0, 0, 1, 0, 0,
                    0, 0, 0, 0, 1, 0,
                    0, 0, 0, 0, 0, 1 );
        }

        void predict(double dt, cv::Mat u);
        void correct(cv::Mat measurement);

        cv::Mat getPosition();
        cv::Mat getSpeed();

    private:

        cv::Mat states_;
        cv::Mat covariance_;

        double processNoise_;
        double observationNoise_;

        static void printMatrix(cv::Mat mat, std::string name);
        static cv::Mat getTransitionMatrix(double dt);  // Matrix A
        static cv::Mat getControlMatrix(double dt) ;    // Matrix B
        static cv::Mat getMeasurementMatrix() ;         // Matrix H
        static cv::Mat getCovarianceObservationNoiseMatrix(double cov);     // Matrix R
        static cv::Mat getCovarianceProcessNoiseMatrix(double sigma, double dt);    // Matrix Q

    };
}
