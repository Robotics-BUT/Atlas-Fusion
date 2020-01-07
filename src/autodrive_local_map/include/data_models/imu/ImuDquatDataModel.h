#pragma once

#include <rtl/Quaternion.h>

#include "data_models/GenericDataModel.h"

namespace AutoDrive::DataModels {

    class ImuDquatDataModel : public GenericDataModel {

    public:

        ImuDquatDataModel(const uint64_t timestamp, const rtl::Quaternion<double>& dquat)
        : GenericDataModel(timestamp)
        , dRotation_(dquat) {
            type_ = DataModelTypes::kImuDquatDataModelType;
        }

        std::string toString() override;

        rtl::Quaternion<double> getDQuat() {return dRotation_;};

    private:

        rtl::Quaternion<double> dRotation_;

    };
}
