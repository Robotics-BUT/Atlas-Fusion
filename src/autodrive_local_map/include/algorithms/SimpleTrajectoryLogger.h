#pragma once

#include <memory>
#include <queue>

#include <rtl/Vector3D.h>
#include <rtl/Quaternion.h>
#include <data_models/imu/ImuGnssDataModel.h>

#include "Context.h"
#include "data_models/gnss/GnssPoseDataModel.h"

#include "data_models/local_map/GlobalPosition.h"
#include "data_models/local_map/LocalPosition.h"

namespace AutoDrive::Algorithms {

    class SimpleTrajectoryLogger {

    public:

        SimpleTrajectoryLogger(Context& context, size_t historyLenght)
        : context_{context}
        , position_{{},{},0}
        , historyLenght_{historyLenght} {

        }

        void onGnssPose(std::shared_ptr<DataModels::GnssPoseDataModel> data);
        void onImuGps(std::shared_ptr<DataModels::ImuGnssDataModel> data);

        DataModels::LocalPosition getPosition() { return position_; };
        std::deque<DataModels::LocalPosition> getPositionHistory() { return positionHistory_; };

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

