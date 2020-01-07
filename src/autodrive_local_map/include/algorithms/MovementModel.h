#pragma once

#include <memory>
#include <queue>
#include <deque>
#include <data_models/imu/ImuDquatDataModel.h>

#include "Kalman1D.h"
#include "Context.h"
#include "data_models/gnss/GnssPoseDataModel.h"
#include "data_models/imu/ImuDquatDataModel.h"
#include "data_models/imu/ImuImuDataModel.h"
#include "data_models/local_map/LocalPosition.h"
#include "data_models/local_map/GlobalPosition.h"

#include "algorithms/ImuDataProcessor.h"

namespace AutoDrive::Algorithms {

    class MovementModel {

    public:

        explicit MovementModel(Context& context, double kalmanProcessNoise, double kalmanObservationNoise)
        : context_{context}
        , kalmanX_{kalmanProcessNoise, kalmanObservationNoise}
        , kalmanY_{kalmanProcessNoise, kalmanObservationNoise}
        , kalmanZ_{kalmanProcessNoise, kalmanObservationNoise}
        , kalmanYaw_{kalmanProcessNoise, kalmanObservationNoise}{ }

        void onGnssPose(std::shared_ptr<DataModels::GnssPoseDataModel> data);
        void onImuImuData(std::shared_ptr<DataModels::ImuImuDataModel> data);
        void onImuDquatData(std::shared_ptr<DataModels::ImuDquatDataModel> data);

        std::deque<DataModels::LocalPosition> getPositionHistory() { return positionHistory_; };

        DataModels::LocalPosition getPosition();

    private:

        Context& context_;

        bool initialized_ = false;
        DataModels::GlobalPosition initPositionGnss_{0,0,0,0};
        std::deque<DataModels::LocalPosition> positionHistory_;

        Kalman1D kalmanX_;
        Kalman1D kalmanY_;
        Kalman1D kalmanZ_;
        Kalman1D kalmanYaw_;

        double roll_{};
        double pitch_{};
        double yaw_{};

        uint64_t lastImuTimestamp_{};
        uint64_t lastDquatTimestamp_{};

        Algorithms::ImuDataProcessor imuProcessor_{};

        double estimateHeading();
        DataModels::GlobalPosition gnssPoseToRootFrame(DataModels::GlobalPosition);
        rtl::Quaternion<double> rpyToQuaternion(double, double, double);
        void quaternionToRPY(rtl::Quaternion<double> q, double& roll, double &pitch, double& yaw);
    };
};
