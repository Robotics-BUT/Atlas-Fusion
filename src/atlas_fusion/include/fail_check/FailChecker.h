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

#include "Context.h"
#include "AbstrackFailChecker.h"
#include "CameraRGBFailChecker.h"
#include "CameraIrFailChecker.h"
#include "GnssFailChecker.h"
#include "LidarFailChecker.h"
#include "ImuFailChecker.h"
#include "RadarTiFileChacker.h"

namespace AtlasFusion::FailCheck {

    /**
     * Fail Checker encapsulates the fail checking for all the sensors via the single API
     */
    class FailChecker {

    public:

        enum class SensorFailCheckID{
            kCameraLeftFront,
            kCameraLeftSide,
            kCameraRightFront,
            kCameraRightSide,
            kCameraIr,
            kLidarLeft,
            kLidarRight,
            kLidarCenter,
            kImu,
            kGnss,
            kRadarTi,
            kErr,
        };

        FailChecker() = delete;

        /**
         * Constructor
         * @param context global services container (time, logging, etc.)
         */
        explicit FailChecker(Context& context)
        : context_{context} {

            failCheckers_[SensorFailCheckID::kCameraLeftFront] = std::dynamic_pointer_cast<AbstrackFailChecker>(std::make_shared<CameraRGBFailChecker>(context));
            failCheckers_[SensorFailCheckID::kCameraLeftSide] = std::dynamic_pointer_cast<AbstrackFailChecker>(std::make_shared<CameraRGBFailChecker>(context));
            failCheckers_[SensorFailCheckID::kCameraRightFront] = std::dynamic_pointer_cast<AbstrackFailChecker>(std::make_shared<CameraRGBFailChecker>(context));
            failCheckers_[SensorFailCheckID::kCameraRightSide] = std::dynamic_pointer_cast<AbstrackFailChecker>(std::make_shared<CameraRGBFailChecker>(context));
            failCheckers_[SensorFailCheckID::kCameraIr] = std::dynamic_pointer_cast<AbstrackFailChecker>(std::make_shared<CameraIrFailChecker>(context));

            failCheckers_[SensorFailCheckID::kLidarLeft] =std::dynamic_pointer_cast<AbstrackFailChecker>(std::make_shared<LidarFailChecker>(context));
            failCheckers_[SensorFailCheckID::kLidarRight] =std::dynamic_pointer_cast<AbstrackFailChecker>(std::make_shared<LidarFailChecker>(context));
            failCheckers_[SensorFailCheckID::kLidarCenter] =std::dynamic_pointer_cast<AbstrackFailChecker>(std::make_shared<LidarFailChecker>(context));

            failCheckers_[SensorFailCheckID::kImu] =std::dynamic_pointer_cast<AbstrackFailChecker>(std::make_shared<ImuFailChecker>(context));
            failCheckers_[SensorFailCheckID::kGnss] =std::dynamic_pointer_cast<AbstrackFailChecker>(std::make_shared<GnssFailChecker>(context));
            failCheckers_[SensorFailCheckID::kRadarTi] = std::dynamic_pointer_cast<AbstrackFailChecker>(std::make_shared<RadarTiFailChecker>(context));
        }

        /**
         * Input pipe for new sensor data
         * @param data sensor data packet
         * @param sensor identifier of the sensor
         */
        void onNewData(std::shared_ptr<DataModels::GenericDataModel> data, SensorFailCheckID sensor);

        /**
         * Returns the reliability of the given sensro
         * @param sensor sensor identificator
         * @return sensor reliability score
         */
        float getSensorStatus(SensorFailCheckID sensor);

        /**
         * Converts sensor frame (string) into Fail Checker specific sensor identifier
         * @param frame name of sensor frame
         * @return Fail Checker specific sensor identifier
         */
        SensorFailCheckID frameToFailcheckID(std::string frame);

    protected:

        Context& context_;
        std::map<SensorFailCheckID, std::shared_ptr<FailCheck::AbstrackFailChecker>> failCheckers_;
    };

}
