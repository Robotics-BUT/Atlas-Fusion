/*
 * Copyright 2020 Brno University of Technology
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom
 * the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT
 * OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
 * OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#pragma once

#include "AbstrackFailChecker.h"

#include "data_models/imu/ImuDquatDataModel.h"
#include "data_models/imu/ImuImuDataModel.h"
#include "data_models/imu/ImuGnssDataModel.h"
#include "data_models/imu/ImuMagDataModel.h"
#include "data_models/imu/ImuPressureDataModel.h"
#include "data_models/imu/ImuTempDataModel.h"
#include "data_models/imu/ImuTimeDataModel.h"

namespace AtlasFusion::FailCheck {

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

