#include "algorithms/Kalman3D.h"

namespace AutoDrive::Algorithms {

    void Kalman3D::predict(double dt, cv::Mat u) {
//        std::cout << " ******** Prediciton ** " << std::endl;
        auto A = getTransitionMatrix(dt);
        auto B = getControlMatrix(dt);
        auto Q = getCovarianceProcessNoiseMatrix(processNoise_, dt);

//        std::cout << "u " << u << std::endl;
        states_ = A * states_ + B * u;
//        printMatrix(states_, "states");
        covariance_ = A * covariance_ * A.t() + Q;
//        printMatrix(covariance_, "covariance_");
    }

    void Kalman3D::correct(cv::Mat measurement) {

//        std::cout << " ******** Prediciton ** " << std::endl;
        auto H = getMeasurementMatrix();
//        printMatrix(H, "H");
        auto R = getCovarianceObservationNoiseMatrix(observationNoise_);
//        printMatrix(R, "R");
        auto K = covariance_ * H.t() / (H * covariance_ * H.t() + R);
//        printMatrix(K, "K");
        states_ = states_ + (K * (measurement - H * states_));
//        printMatrix(states_, "states_");
        covariance_ = ( cv::Mat::eye(6,6, H.type()) - K * H) * covariance_;
//        printMatrix(covariance_, "covariance_");
    }

    cv::Mat Kalman3D::getPosition() {
        cv::Mat pose = (cv::Mat_<double>(3, 1) <<
               states_.at<double>(0),
               states_.at<double>(1),
               states_.at<double>(2)); // v
        return pose;
    }

    cv::Mat Kalman3D::getSpeed() {
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