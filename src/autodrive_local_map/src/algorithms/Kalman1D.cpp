#include "algorithms/Kalman1D.h"

namespace AutoDrive::Algorithms {

    void Kalman1D::predict(double dt, double u) {
        auto A = getTransitionMatrix(dt);
        auto B = getControlMatrix(dt);
        auto Q = getCovarianceProcessNoiseMatrix(processNoise_, dt);

        states_ = A * states_ + B * u;
        covariance_ = A * covariance_ * A.t() + Q;
    }

    void Kalman1D::correct(cv::Mat measurement) {

        auto H = getMeasurementMatrix();
        auto R = getCovarianceObservationNoiseMatrix(observationNoise_);
        auto K = covariance_ * H.t() / (H * covariance_ * H.t() + R);
        states_ = states_ + (K * (measurement - H * states_));
        covariance_ = ( cv::Mat::eye(2,2, H.type()) - K * H) * covariance_;
    }

    double Kalman1D::getPosition() const {
        auto pose = states_.at<double>(0);
        return pose;
    }

    double Kalman1D::getVelocity() const {
        auto speed = states_.at<double>(1);
        return speed;
    }

    void Kalman1D::setPosition(double position) {
        states_.at<double>(0) = position;
    }

    void Kalman1D::setVelocity(double speed) {
        states_.at<double>(1) = speed;
    }


    void Kalman1D::printMatrix(cv::Mat mat, std::string name) {
        std::cout << " *** " << name << " *** " << std::endl;
        for(int i = 0 ; i < mat.rows ; i++) {
            for (int j = 0 ; j < mat.cols ; j++) {
                std::cout << mat.at<double>(j + i*mat.cols) << " ";
            }
            std::cout << std::endl;
        }
    }

    cv::Mat Kalman1D::getTransitionMatrix(double dt) {
        cv::Mat transitionMatrix = (cv::Mat_<double>(2, 2) <<
                1, dt,  // p
                0,  1); // v
        return transitionMatrix;
    }

    cv::Mat Kalman1D::getControlMatrix(double dt) {
        cv::Mat transitionMatrix = (cv::Mat_<double>(2, 1) <<
                0.5*dt*dt,
                dt );

        return transitionMatrix;
    }

    cv::Mat Kalman1D::getMeasurementMatrix() {
        cv::Mat measurementMatrix = (cv::Mat_<double>(2, 2) <<
                1, 0,
                0, 0);

        return measurementMatrix;
    }

    cv::Mat Kalman1D::getCovarianceProcessNoiseMatrix(double sigma, double dt) {

        auto s4 = pow(dt,4) * sigma;
        auto s3 = pow(dt,3) * sigma;
        auto s2 = pow(dt,2) * sigma;
        cv::Mat covarianceProcessNoiseMatrix = (cv::Mat_<double>(2, 2) <<
                s4, s3,
                s3, s2);

        return covarianceProcessNoiseMatrix;
    }

    cv::Mat Kalman1D::getCovarianceObservationNoiseMatrix(double cov) {
        cv::Mat covarianceObservationNoiseMatrix = (cv::Mat_<double>(2, 2) <<
                cov, cov,
                cov, cov);
        return covarianceObservationNoiseMatrix;
    }

}