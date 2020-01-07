#pragma once

#include "data_models/GenericDataModel.h"

#include <rtl/Vector3D.h>
#include <rtl/Quaternion.h>

namespace AutoDrive::DataModels {

    class ImuImuDataModel : public GenericDataModel {

    public:

        ImuImuDataModel(uint64_t timestamp, rtl::Vector3D<double> linAcc, rtl::Vector3D<double> angVel, rtl::Quaternion<double> orient)
        : GenericDataModel(timestamp)
        , linearAcc_(linAcc)
        , angularVel_(angVel)
        , orientation_(orient) {
            type_ = DataModelTypes::kImuImuDataModelType;
        };

        std::string toString() override;

        rtl::Vector3D<double> getLinearAcc() {return linearAcc_;};
        rtl::Vector3D<double> getAngularVel() {return angularVel_;};
        rtl::Quaternion<double> getOrientation() {return orientation_;};

    private:

        rtl::Vector3D<double> linearAcc_;
        rtl::Vector3D<double> angularVel_;
        rtl::Quaternion<double> orientation_;


    };
}
