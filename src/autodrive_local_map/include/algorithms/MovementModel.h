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
#include "algorithms/QuadratureFilter.h"

namespace AutoDrive::Algorithms {

    class MovementModel {

    public:

        explicit MovementModel(Context& context, double kalmanProcessNoise, double kalmanObservationNoise)
        : context_{context}
        , kalmanX_{kalmanProcessNoise, kalmanObservationNoise}
        , kalmanY_{kalmanProcessNoise, kalmanObservationNoise}
        , kalmanZ_{kalmanProcessNoise, kalmanObservationNoise}
        //, kalmanYaw_{kalmanProcessNoise, kalmanObservationNoise}
        {

        }

        void onGnssPose(std::shared_ptr<DataModels::GnssPoseDataModel> data);
        void onImuImuData(std::shared_ptr<DataModels::ImuImuDataModel> data);
        void onImuDquatData(std::shared_ptr<DataModels::ImuDquatDataModel> data);

        std::deque<DataModels::LocalPosition> getPositionHistory() { return positionHistory_; };

        DataModels::LocalPosition getPosition() const;
        double getSpeed() const;
        rtl::Vector3D<double> getVectorSpeed() const;

        double getHeading() const;
        rtl::Quaternion<double> getOrientation() const;

        DataModels::LocalPosition estimatePositionInTime(uint64_t time);
        static DataModels::LocalPosition interpolateLocalPosition( DataModels::LocalPosition& begin, DataModels::LocalPosition& end, float ratio);

        [[deprecated]] float getSpeedAzimOffset();

    private:

        Context& context_;

        bool poseInitialized_ = false;
        bool headingInitialized_ = false;

        DataModels::GlobalPosition initPositionGnss_{0,0,0,0};
        std::deque<DataModels::LocalPosition> positionHistory_;

        Kalman1D kalmanX_;
        Kalman1D kalmanY_;
        Kalman1D kalmanZ_;

        rtl::Quaternion<double> orientation_;

        uint64_t lastImuTimestamp_{};
        uint64_t lastDquatTimestamp_{};
        uint64_t currentTime{};

        Algorithms::ImuDataProcessor imuProcessor_{};

        uint64_t lastValidGnssHeadingTimestamp_{};

        double estimateHeading() const;
        double estimateSpeedHeading() const;

        DataModels::GlobalPosition gnssPoseToRootFrame(DataModels::GlobalPosition);
        rtl::Quaternion<double> rpyToQuaternion(double, double, double);
        void quaternionToRPY(rtl::Quaternion<double> q, double& roll, double &pitch, double& yaw) const;

        double getSpeedGainFactor(double speed) const;
        double azimuthToHeading(double azimuth) const;
    };
};
