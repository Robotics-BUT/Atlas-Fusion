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

#include <memory>
#include <queue>

#include <rtl/Core.h>
#include <data_models/imu/ImuGnssDataModel.h>

#include "Context.h"
#include "data_models/gnss/GnssPoseDataModel.h"

#include "data_models/local_map/GlobalPosition.h"
#include "data_models/local_map/LocalPosition.h"

namespace AtlasFusion::Algorithms {

    /**
     * Simple Trajectory Logger is dedicated to convert GNSS data into local position w.r.t. the local anchor and to
     * remember this defined lenght of history of the position measurements
     */
    class SimpleTrajectoryLogger {

    public:

        /**
         * Constructor
         * @param context global services container (time, logging, etc.)
         * @param historyLength the number of remembered points in the history
         */
        SimpleTrajectoryLogger(Context& context, size_t historyLenght)
        : context_{context}
        , position_{{0.0, 0.0, 0.0}, rtl::Quaternion<double>::identity(), 0}
        , historyLenght_{historyLenght} {

        }

        /**
         * Input for GNSS Receiver data
         * @param data global position data
         */
        void onGnssPose(std::shared_ptr<DataModels::GnssPoseDataModel> data);

        /**
         * Input for IMU GPS receiver
         * @param data global position data
         */
        void onImuGps(std::shared_ptr<DataModels::ImuGnssDataModel> data);

        /**
         * Getter for last position
         * @return last position in history
         */
        DataModels::LocalPosition getPosition() { return position_; };

        /**
         * Getter for entire position history
         * @return full history position
         */
        std::deque<DataModels::LocalPosition> getPositionHistory() { return positionHistory_; };

        /**
         * Setter for altitude for last latest position
         * @param alt current altitude
         */
        void setAltitude(double alt) {position_.setZ(alt);};

    private:
        Context& context_;
        DataModels::LocalPosition position_;


        bool initialized_ = false;
        DataModels::GlobalPosition initPositionGnss_{0,0,0,0};
        std::deque<DataModels::LocalPosition> positionHistory_;

        size_t historyLenght_;

        DataModels::GlobalPosition gnssPoseToRootFrame(const DataModels::GlobalPosition);
        rtl::Quaternion<double> rpyToQuaternion(double, double, double);
    };
}

