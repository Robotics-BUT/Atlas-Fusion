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

#pragma once

#include <opencv2/video/tracking.hpp>
#include <iostream>

namespace AtlasFusion::Algorithms {

    /**
     * Wrapper around the OpenCV Kalman filter implementations. Models position and velocity in 1D space
     */

    class Kalman1D {

    public:

        /**
         * Constructor
         * @param processNoise the noise of the modeled process
         * @param observationNoise the noise of the measured data
         */
        Kalman1D(double processNoise, double observationNoise)
        : processNoise_(processNoise)
        , observationNoise_(observationNoise) {
            states_ = (cv::Mat_<double>(2, 1) <<
                    0,
                    0);
            covariance_ = (cv::Mat_<double>(2, 2) <<
                    1, 0,
                    0, 1);
        }

        /**
         * Prediction phase of the Kalman Filter
         * @param dt time period from last update
         * @param u action input (acceleration in given axis)
         */
        void predict(double dt, double u);

        /**
         * Corection phase of the Kalman Filter
         * @param measurement measurements of the inner states
         */
        void correct(cv::Mat measurement);

        /**
         * Getter for a position inner state
         * @return modeled position
         */
        double getPosition() const;

        /**
         * Getter for a velocity inner state
         * @return modeles velocity
         */
        double getVelocity() const;

        /**
         * Setter for a modeled position
         * @param position position to be forced into the model
         */
        void setPosition(double position);

        /**
         * Setter for a modeled velocity
         * @param velocity velocity to be forced into the model
         */
        void setVelocity(double velocity);

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
