#pragma once

#include <memory>
#include <queue>

#include <rtl/Core.h>
#include <data_models/imu/ImuGnssDataModel.h>

#include "Context.h"
#include "data_models/gnss/GnssPoseDataModel.h"

#include "data_models/local_map/GlobalPosition.h"
#include "data_models/local_map/LocalPosition.h"

namespace AutoDrive::Algorithms {

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

