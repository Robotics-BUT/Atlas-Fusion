#pragma once

#include <deque>

#include "Kalman1D.h"
#include "Context.h"

#include "data_models/gnss/GnssPoseDataModel.h"
#include "data_models/imu/ImuDquatDataModel.h"
#include "data_models/imu/ImuImuDataModel.h"
#include "data_models/local_map/LocalPosition.h"
#include "data_models/local_map/GlobalPosition.h"

#include "algorithms/ImuDataProcessor.h"

namespace AutoDrive::Algorithms {

    class SelfModel {

    public:

        SelfModel(Context& context, double kalmanProcessNoise, double kalmanObservationNoise)
        : context_{context}
        , kalmanX_{kalmanProcessNoise, kalmanObservationNoise}
        , kalmanY_{kalmanProcessNoise, kalmanObservationNoise}
        , kalmanZ_{kalmanProcessNoise, kalmanObservationNoise}
        {

        }

        /// Hooks
        void onGnssPose(std::shared_ptr<DataModels::GnssPoseDataModel> data);
        void onImuImuData(std::shared_ptr<DataModels::ImuImuDataModel> data);
        void onImuDquatData(std::shared_ptr<DataModels::ImuDquatDataModel> data);

        /// Getters

        // Position
        DataModels::LocalPosition getPosition() const;
        std::deque<DataModels::LocalPosition> getPositionHistory();

        // Speed
        double getSpeedScalar() const;
        rtl::Vector3D<double> getSpeedVector() const;

        // Acc
        double getAvgAccScalar() const;
        rtl::Vector3D<double> getAvgAcceleration() const;

        // Orientation
        rtl::Quaternion<double> getOrientation() const;
        double getHeading() const;

        std::string getTelemetryString() const;

        DataModels::LocalPosition estimatePositionInTime(uint64_t time);

    private:

        Context& context_;

        Kalman1D kalmanX_;
        Kalman1D kalmanY_;
        Kalman1D kalmanZ_;
        rtl::Quaternion<double> orientation_;

        bool poseInitialized_ = false;
        bool orientationInitialized_ = false;

        Algorithms::ImuDataProcessor imuProcessor_{};

        DataModels::GlobalPosition anchorPose{0,0,0,0};
        std::deque<DataModels::LocalPosition> positionHistory_;
        std::deque<std::pair<rtl::Vector3D<double>, uint64_t >> accHistory_;

        int validHeadingCounter_ = 0;

        uint64_t lastGnssTimestamp_{};
        uint64_t lastImuTimestamp_{};
        uint64_t lastDquatTimestamp_{};


        std::pair<double, float> validHeading(std::shared_ptr<DataModels::GnssPoseDataModel> data);
        std::pair<double, float> speedHeading();
        std::pair<double, float> fuseHeadings(std::shared_ptr<DataModels::GnssPoseDataModel> data);
        float estimateSlerpFactor(float, float);
        void updateOrientation(double heading);

        DataModels::GlobalPosition gnssPoseToRootFrame(DataModels::GlobalPosition);

        rtl::Vector3D<double> removeGravitationalForceFromLinAcc( std::shared_ptr<DataModels::ImuImuDataModel> data);
        uint64_t getCurrentTime() const;
    };
}

