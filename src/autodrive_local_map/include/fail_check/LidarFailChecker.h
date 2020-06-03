#pragma once

#include "AbstrackFailChecker.h"
#include "data_models/lidar/LidarScanDataModel.h"

namespace AutoDrive::FailCheck {

    /**
     * Validates LiDAR point cloud scans. Currently bypassed.
     */
    class LidarFailChecker : public AbstrackFailChecker{

    public:

        LidarFailChecker() = delete;

        /**
         * Constructor
         * @param context cantainer for global services (timestamps. logging, etc.)
         */
        LidarFailChecker(Context& context)
                : AbstrackFailChecker{context}
        {

        }

        /**
         * Pipe to provide new sensor data into the LiDAR Fail Checker
         * @param data point cloud scan
         */
        void onNewData(std::shared_ptr<DataModels::LidarScanDataModel>);

    private:

    };
}

