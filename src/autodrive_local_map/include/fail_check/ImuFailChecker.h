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

    /**
     * Validates IMU data packets. Currently bypassed.
     */
    class ImuFailChecker : public AbstrackFailChecker{

    public:

        ImuFailChecker() = delete;

        /**
         * Constructor
         * @param context cantainer for global services (timestamps. logging, etc.)
         */
        ImuFailChecker(Context& context)
        : AbstrackFailChecker{context}
        {

        }

        /**
         * Input method for IMU Delta Quaternion Data
         * @param data orientation diff of the IMU sensor
         */
        void onNewData(std::shared_ptr<DataModels::ImuDquatDataModel> data);

        /**
         * Input method for IMU data that contains the linear acceleration, angular velocity and absolute orientation
         * info
         * @param data IMU data frame
         */
        void onNewData(std::shared_ptr<DataModels::ImuImuDataModel> data);

        /**
         * Input method for IMU global position data frame
         * @param data global position data
         */
        void onNewData(std::shared_ptr<DataModels::ImuGnssDataModel> data);

        /**
         * Input method for IMU magnetic field measurement
         * @param data magnetic field intensity data
         */
        void onNewData(std::shared_ptr<DataModels::ImuMagDataModel> data);

        /**
         * Input method for IMU atmospheric pressure measurement
         * @param data atmospheric pressure
         */
        void onNewData(std::shared_ptr<DataModels::ImuPressureDataModel> data);

        /**
         * Input method for IMU sensor's inner temperature value
         * @param data innter IMU temperature
         */
        void onNewData(std::shared_ptr<DataModels::ImuTempDataModel> data);

        /**
         * Input method for IMU time received by GNSS receiver
         * @param data IMU time frame received via GNSS
         */
        void onNewData(std::shared_ptr<DataModels::ImuTimeDataModel> data);

    private:

    };
}

