#pragma once

#include <rtl/Core.h>

#include "data_models/GenericDataModel.h"

namespace AutoDrive::DataModels {

    /**
     * Imu Delta Quaterion represents the absolute orientation diff from last measurement
     */
    class ImuDquatDataModel : public GenericDataModel {

    public:

        /**
         * Constructor
         * @param timestamp measurement timestamp
         * @param dquat past orientation * dquat = current orientation
         */
        ImuDquatDataModel(const uint64_t timestamp, const rtl::Quaternion<double>& dquat)
        : GenericDataModel(timestamp)
        , dRotation_(dquat) {
            type_ = DataModelTypes::kImuDquatDataModelType;
        }

        std::string toString() override;

        /**
         * Orientation delta quaternion getter
         * @return delta quaternion
         */
        rtl::Quaternion<double> getDQuat() {return dRotation_;};

    private:

        rtl::Quaternion<double> dRotation_;

    };
}
