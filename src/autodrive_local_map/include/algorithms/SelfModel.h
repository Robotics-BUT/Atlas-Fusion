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

    /**
     * Self Model takes care about the sensor-equipped motion modeling. It combines the GNNS and IMU measurements
     * to estimate best effort position that is later used in the mapping apgorithms
     */
    class SelfModel {

    public:

        /**
         * Constructor
         * @param context global services container (time, logging, TF tree, etc.)
         * @param kalmanProcessNoise noise of the modeled process (feeds Kalman Filter)
         * @param kalmanObservationNoise measurement noise (feeds Kalman Filter)
         */
        SelfModel(Context& context, double kalmanProcessNoise, double kalmanObservationNoise)
        : context_{context}
        , kalmanX_{kalmanProcessNoise, kalmanObservationNoise}
        , kalmanY_{kalmanProcessNoise, kalmanObservationNoise}
        , kalmanZ_{kalmanProcessNoise, kalmanObservationNoise}
        {

        }

        /**
         * Input pipe for new GNSS data
         * @param data GNSS data frame
         */
        void onGnssPose(std::shared_ptr<DataModels::GnssPoseDataModel> data);

        /**
         * Input pipe for new IMU data
         * @param data IMU data frame
         */
        void onImuImuData(std::shared_ptr<DataModels::ImuImuDataModel> data);

        /**
         * Input pipe for new IMU (orientation change) data
         * @param data IMU (orientation change) data frame
         */
        void onImuDquatData(std::shared_ptr<DataModels::ImuDquatDataModel> data);

        /**
         * Getter for current position in the global coordinate systems
         * @return current local position
         */
        DataModels::LocalPosition getPosition() const;

        /**
         * Returns history of N position points from history.
         * @return history of agent's positions
         */
        std::deque<DataModels::LocalPosition> getPositionHistory();

        /**
         * Getter for scalar speed in this moment
         * @return current speed
         */
        double getSpeedScalar() const;

        /**
         * Getter for velocity vector in this moment
         * @return current velocity
         */
        rtl::Vector3D<double> getSpeedVector() const;

        /**
         * Total acc force acting on the agent
         * @return current acc scalar
         */
        double getAvgAccScalar() const;

        /**
         * Total vector of the acc forces acting on the agent
         * @return current acc vector
         */
        rtl::Vector3D<double> getAvgAcceleration() const;

        /**
         * Current agent's orientation
         * @return quaternion that represents the current agent's orientation w.r.t. the initialization anchor
         */
        rtl::Quaternion<double> getOrientation() const;

        /**
         * Current agent's azimuth
         * @return estimated azimut of the agent (car)
         */
        double getHeading() const;

        /**
         * Generates telemetry string for the visualization purposes
         * @return string that contains telemetry informations
         */
        std::string getTelemetryString() const;

        /**
         * Interpolates agent's position in the history between the samples
         * @param time given time in history
         * @return interpolated position in the global coordinates system
         */
        DataModels::LocalPosition estimatePositionInTime(uint64_t time);

    private:

        Context& context_;

        Kalman1D kalmanX_;
        Kalman1D kalmanY_;
        Kalman1D kalmanZ_;
        rtl::Quaternion<double> orientation_ = rtl::Quaternion<double>::identity();

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

