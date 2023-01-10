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

#include <deque>
#include <sunset.h>
#include <pcl/point_types.h>


#include "Kalman1D.h"
#include "Context.h"

#include "data_models/gnss/GnssPoseDataModel.h"
#include "data_models/imu/ImuDquatDataModel.h"
#include "data_models/imu/ImuImuDataModel.h"
#include "data_models/local_map/LocalPosition.h"
#include "data_models/local_map/GlobalPosition.h"

#include "algorithms/ImuDataProcessor.h"
#include "data_models/gnss/GnssTimeDataModel.h"
#include "algorithms/pointcloud/PointCloudProcessor.h"

#define TIMEZONE_OFFSET 1

namespace AutoDrive::Algorithms {

    /**
     * Environmental model takes care of calculating and holding the environmental state which has effect on every sensor on the vehicle
     * This state holds information about weather and time flags
     */
    class EnvironmentalModel {

    public:

        /**
         * Constructor
         * @param context global services container (time, logging, TF tree, etc.)
         */
        explicit EnvironmentalModel(Context &context, PointCloudProcessor &pointCloudProcessor) : context_{context},
                                                                                                  pointCloudProcessor_{pointCloudProcessor} {}

        void onGnssTime(const std::shared_ptr<DataModels::GnssTimeDataModel> &timeData);

        void onGnssPose(const std::shared_ptr<DataModels::GnssPoseDataModel> &poseData);

        void onDetectionROI(const pcl::PointCloud<pcl::PointXYZ>::Ptr &detectionROI);

        void calculateSunriseAndSunsetTimes();

        [[nodiscard]] bool getIsDaylight() const { return isDaylight_; };

        void setIsWetRoad(bool isWetRoad) { isWetRoad_ = isWetRoad; }

        bool getIsWetRoad() { return isWetRoad_; }

        bool getIsSkyOccluded();

    private:

        Context &context_;
        PointCloudProcessor &pointCloudProcessor_;
        SunSet sunCalc = SunSet();

        uint64_t timestamp_ = 0;
        uint32_t year_ = 0, month_ = 0, day_ = 0, hours_ = 0, minutes_ = 0, seconds_ = 0, daylightSavingsTimeOffset_ = 0;
        double latitude_ = 0, longitude_ = 0, timezone_ = 0;

        double sunrise_ = 0, sunset_ = 0;
        bool isDaylight_ = false;
        bool isWetRoad_ = false;
        bool isSkyOccluded = false;

        uint64_t skyOcclusionTime_;
    };
}

