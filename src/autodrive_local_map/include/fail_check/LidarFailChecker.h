#pragma once

#include "AbstrackFailChecker.h"
#include "data_models/lidar/LidarScanDataModel.h"

namespace AutoDrive::FailCheck {

    class LidarFailChecker : public AbstrackFailChecker{

    public:

        LidarFailChecker() = delete;

        LidarFailChecker(Context& context)
                : AbstrackFailChecker{context}
        {

        }

        void onNewData(std::shared_ptr<DataModels::LidarScanDataModel>);

    private:

    };
}

