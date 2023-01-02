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
#include "AbstractFailChecker.h"
#include "CameraRGBFailChecker.h"
#include "CameraIrFailChecker.h"
#include "GnssFailChecker.h"
#include "LidarFailChecker.h"
#include "ImuFailChecker.h"
#include "RadarTiFileChacker.h"

namespace AutoDrive::FailCheck {

    /**
     * Fail Checker encapsulates the fail checking for all the sensors via the single API
     */
    class FailChecker {

    public:

        FailChecker() = delete;

        /**
         * Constructor
         * @param context global services container (time, logging, etc.)
         */
        explicit FailChecker(Context &context, const Algorithms::SelfModel &selfModel) : context_{context}, selfModel_{selfModel} {

            failCheckers_[FrameType::kCameraLeftFront] = std::dynamic_pointer_cast<AbstractFailChecker>(std::make_shared<CameraRGBFailChecker>(context, selfModel_));
            failCheckers_[FrameType::kCameraLeftSide] = std::dynamic_pointer_cast<AbstractFailChecker>(std::make_shared<CameraRGBFailChecker>(context, selfModel_));
            failCheckers_[FrameType::kCameraRightFront] = std::dynamic_pointer_cast<AbstractFailChecker>(std::make_shared<CameraRGBFailChecker>(context, selfModel_));
            failCheckers_[FrameType::kCameraRightSide] = std::dynamic_pointer_cast<AbstractFailChecker>(std::make_shared<CameraRGBFailChecker>(context, selfModel_));
            failCheckers_[FrameType::kCameraIr] = std::dynamic_pointer_cast<AbstractFailChecker>(std::make_shared<CameraIrFailChecker>(context, selfModel_));

            failCheckers_[FrameType::kLidarLeft] = std::dynamic_pointer_cast<AbstractFailChecker>(std::make_shared<LidarFailChecker>(context, selfModel_));
            failCheckers_[FrameType::kLidarRight] = std::dynamic_pointer_cast<AbstractFailChecker>(std::make_shared<LidarFailChecker>(context, selfModel_));
            failCheckers_[FrameType::kLidarCenter] = std::dynamic_pointer_cast<AbstractFailChecker>(std::make_shared<LidarFailChecker>(context, selfModel_));

            failCheckers_[FrameType::kImu] = std::dynamic_pointer_cast<AbstractFailChecker>(std::make_shared<ImuFailChecker>(context, selfModel_));
            failCheckers_[FrameType::kGnssAntennaRear] = std::dynamic_pointer_cast<AbstractFailChecker>(std::make_shared<GnssFailChecker>(context, selfModel_));
            failCheckers_[FrameType::kRadarTi] = std::dynamic_pointer_cast<AbstractFailChecker>(std::make_shared<RadarTiFailChecker>(context, selfModel_));
        }

        /**
         * Input pipe for new sensor data
         * @param data sensor data packet
         * @param sensor identifier
         */
        void onNewData(const std::shared_ptr<DataModels::GenericDataModel> &data, const FrameType &sensor);

        /**
         * Returns the reliability of the given sensor
         * @param sensor identifier
         * @return sensor reliability score
         */
        float getSensorStatus(const FrameType &sensor);

    protected:

        Context &context_;
        const Algorithms::SelfModel &selfModel_;
        std::map<FrameType, std::shared_ptr<FailCheck::AbstractFailChecker>> failCheckers_;
    };

}
