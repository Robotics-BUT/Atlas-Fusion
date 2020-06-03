#pragma once

#include "data_models/GenericDataModel.h"

#include <rtl/Vector3D.h>
#include <rtl/Quaternion.h>

namespace AutoDrive::DataModels {

    /**
     * IMU-IMU Data Model represents the Xsens data packet that provides linear accelerations, angular velocities and
     * absolute orientiation, all in 3D
     */
    class ImuImuDataModel : public GenericDataModel {

    public:

        /**
         * Constructor
         * @param timestamp measurement timestamp
         * @param linAcc 3D Linear acceleration
         * @param angVel 3D angular velocities
         * @param orient absolute IMU orientation
         */
        ImuImuDataModel(uint64_t timestamp, rtl::Vector3D<double> linAcc, rtl::Vector3D<double> angVel, rtl::Quaternion<double> orient)
        : GenericDataModel(timestamp)
        , linearAcc_(linAcc)
        , angularVel_(angVel)
        , orientation_(orient) {
            type_ = DataModelTypes::kImuImuDataModelType;
        };

        std::string toString() override;

        /**
         * 3D Linear acceleration vector getter
         * @return 3D linear acceleration
         */
        rtl::Vector3D<double> getLinearAcc() {return linearAcc_;};

        /**
         * 3-axis angular velocities getter
         * @return 3D angular velocities
         */
        rtl::Vector3D<double> getAngularVel() {return angularVel_;};

        /**
         * Absolute IMU orientation getter
         * @return IMU orientation
         */
        rtl::Quaternion<double> getOrientation() {return orientation_;};

    private:

        rtl::Vector3D<double> linearAcc_;
        rtl::Vector3D<double> angularVel_;
        rtl::Quaternion<double> orientation_;


    };
}
