/*
 * Copyright 2020 Brno University of Technology
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom
 * the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT
 * OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
 * OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include "algorithms/Kalman3D.h"

namespace AtlasFusion::Algorithms {

    void Kalman3D::predict(double dt, cv::Mat u) {
        auto A = getTransitionMatrix(dt);
        auto B = getControlMatrix(dt);
        auto Q = getCovarianceProcessNoiseMatrix(processNoise_, dt);

        states_ = A * states_ + B * u;
        covariance_ = A * covariance_ * A.t() + Q;
    }

    void Kalman3D::correct(cv::Mat measurement) {

        auto H = getMeasurementMatrix();
        auto R = getCovarianceObservationNoiseMatrix(observationNoise_);
        auto K = covariance_ * H.t() / (H * covariance_ * H.t() + R);
        states_ = states_ + (K * (measurement - H * states_));
        covariance_ = ( cv::Mat::eye(6,6, H.type()) - K * H) * covariance_;
    }

    cv::Mat Kalman3D::getPosition() {
        cv::Mat pose = (cv::Mat_<double>(3, 1) <<
               states_.at<double>(0),
               states_.at<double>(1),
               states_.at<double>(2)); // v
        return pose;
    }

    cv::Mat Kalman3D::getVelocity() {
        cv::Mat speed = (cv::Mat_<double>(3, 1) <<
                states_.at<double>(3),
                states_.at<double>(4),
                states_.at<double>(5)); // v
        return speed;
    }

    void Kalman3D::printMatrix(cv::Mat mat, std::string name) {
        std::cout << " *** " << name << " *** " << std::endl;
        for(int i = 0 ; i < mat.rows ; i++) {
            for (int j = 0 ; j < mat.cols ; j++) {
                std::cout << mat.at<double>(j + i*mat.cols) << " ";
            }
            std::cout << std::endl;
        }
    }

    cv::Mat Kalman3D::getTransitionMatrix(double dt) {
        cv::Mat transitionMatrix = (cv::Mat_<double>(6, 6) <<
                1,  0,  0, dt,  0,  0,
                0,  1,  0,  0, dt,  0,
                0,  0,  1,  0,  0, dt,
                0,  0,  0,  1,  0,  0,
                0,  0,  0,  0,  1,  0,
                0,  0,  0,  0,  0,  1);
        return transitionMatrix;
    }

    cv::Mat Kalman3D::getControlMatrix(double dt) {
        cv::Mat controlMatrix = (cv::Mat_<double>(6, 3) <<
                0.5*dt*dt,         0,         0,
                        0, 0.5*dt*dt,         0,
                        0,         0, 0.5*dt*dt,
                       dt,         0,         0,
                        0,        dt,         0,
                        0,         0,        dt);

        return controlMatrix;
    }

    cv::Mat Kalman3D::getMeasurementMatrix() {
        cv::Mat measurementMatrix = (cv::Mat_<double>(6, 6) <<
                1, 0, 0, 0, 0, 0,
                0, 1, 0, 0, 0, 0,
                0, 0, 1, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0);

        return measurementMatrix;
    }

    cv::Mat Kalman3D::getCovarianceProcessNoiseMatrix(double sigma, double dt) {

        auto s4 = pow(dt,4) * sigma;
        auto s3 = pow(dt,3) * sigma;
        auto s2 = pow(dt,2) * sigma;
        cv::Mat covarianceProcessNoiseMatrix = (cv::Mat_<double>(6, 6) <<
                s4,  0,  0, s3,  0,  0,
                 0, s4,  0,  0, s3,  0,
                 0,  0, s4,  0,  0, s3,
                s3,  0,  0, s2,  0,  0,
                 0, s3,  0,  0, s2,  0,
                 0,  0, s3,  0,  0, s2);

        return covarianceProcessNoiseMatrix;
    }

    cv::Mat Kalman3D::getCovarianceObservationNoiseMatrix(double cov) {
        cv::Mat covarianceObservationNoiseMatrix = (cv::Mat_<double>(6, 6) <<
                cov, cov, cov, cov, cov, cov,
                cov, cov, cov, cov, cov, cov,
                cov, cov, cov, cov, cov, cov,
                cov, cov, cov, cov, cov, cov,
                cov, cov, cov, cov, cov, cov,
                cov, cov, cov, cov, cov, cov);
        return covarianceObservationNoiseMatrix;
    }

}