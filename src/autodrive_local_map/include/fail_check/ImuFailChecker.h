#pragma once

#include "AbstrackFailChecker.h"

#include "data_models/imu/ImuDquatDataModel.h"
#include "data_models/imu/ImuImuDataModel.h"
#include "data_models/imu/ImuGnssDataModel.h"
#include "data_models/imu/ImuMagDataModel.h"
#include "data_models/imu/ImuPressureDataModel.h"
#include "data_models/imu/ImuTempDataModel.h"
#include "data_models/imu/ImuTimeDataModel.h"

namespace AutoDrive::FailCheck {

    class ImuFailChecker : public AbstrackFailChecker{

    public:

        ImuFailChecker() = delete;

        ImuFailChecker(Context& context)
        : AbstrackFailChecker{context}
        {

        }

        void onNewData(std::shared_ptr<DataModels::ImuDquatDataModel>);
        void onNewData(std::shared_ptr<DataModels::ImuImuDataModel>);
        void onNewData(std::shared_ptr<DataModels::ImuGnssDataModel>);
        void onNewData(std::shared_ptr<DataModels::ImuMagDataModel>);
        void onNewData(std::shared_ptr<DataModels::ImuPressureDataModel>);
        void onNewData(std::shared_ptr<DataModels::ImuTempDataModel>);
        void onNewData(std::shared_ptr<DataModels::ImuTimeDataModel>);

    private:

    };
}

