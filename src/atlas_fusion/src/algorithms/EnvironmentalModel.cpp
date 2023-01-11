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

#include "algorithms/EnvironmentalModel.h"

namespace AutoDrive::Algorithms {

    void EnvironmentalModel::onGnssTime(const std::shared_ptr<DataModels::GnssTimeDataModel> &timeData) {
        timestamp_ = timeData->getTimestamp();
        year_ = timeData->getYear();
        month_ = timeData->getMonth();
        day_ = timeData->getDay();
        hours_ = timeData->getHour();
        minutes_ = timeData->getMinute();
        seconds_ = timeData->getSec();

        daylightSavingsTimeOffset_ = 0.0;
        if (month_ > 3 && month_ < 10) daylightSavingsTimeOffset_ = 1.0;
        if (month_ == 3 && day_ >= 26 && hours_ >= 2) daylightSavingsTimeOffset_ = 1.0;
        if (month_ == 10 && day_ <= 29 && hours_ <= 3) daylightSavingsTimeOffset_ = 1.0;
        hours_ += int(daylightSavingsTimeOffset_);
    }

    void EnvironmentalModel::onGnssPose(const std::shared_ptr<DataModels::GnssPoseDataModel> &poseData) {
        latitude_ = poseData->getLatitude();
        longitude_ = poseData->getLongitude();
        altitude_ = poseData->getAltitude();
        timezone_ = TIMEZONE_OFFSET;
    }

    void EnvironmentalModel::onDetectionROI(const pcl::PointCloud<pcl::PointXYZ>::Ptr &detectionROI) {
        auto tunnelCheckROI = pointCloudProcessor_.getPointCloudCutout(detectionROI, rtl::BoundingBox3D<float>{rtl::Vector3D<float>{-40.f, -1.f, 3.f},
                                                                                                               rtl::Vector3D<float>{40.f, 3.f, 10.f}});
        if (tunnelCheckROI->size() > 500) {
            skyOcclusionTime_ = timestamp_;
            isSkyOccluded_ = true;
        }
    }

    bool EnvironmentalModel::getIsSkyOccluded() {
        double diff = double(timestamp_ - skyOcclusionTime_) * 1e-9;
        if (diff > 5) {
            isSkyOccluded_ = false;
        }
        return isSkyOccluded_;
    }

    void EnvironmentalModel::calculateSunriseAndSunsetTimes() {
        if (latitude_ == 0 || longitude_ == 0 || year_ == 0 || month_ == 0 || day_ == 0)
            return;

        sunCalc.setCurrentDate(int(year_), int(month_), int(day_));
        sunCalc.setPosition(latitude_, longitude_, timezone_ + daylightSavingsTimeOffset_);
        sunrise_ = sunCalc.calcSunrise() / 60.0;
        sunset_ = sunCalc.calcSunset() / 60.0;
        double h = hours_ + TIMEZONE_OFFSET + (minutes_ / 60.0);
        isDaylight_ = h < sunset_ && h > sunrise_;
    }

    std::string EnvironmentalModel::getStatusString() {
        std::string statusStr;
        statusStr += std::string("Environmental model") + "\n";
        statusStr += "Date: " + std::to_string(day_) + "." + std::to_string(month_) + "." + std::to_string(year_) + "\n";
        statusStr += "Time: " + std::to_string(hours_) + ":" + std::to_string(minutes_) + ":" + std::to_string(seconds_) + "\n";
        statusStr += "Position: " "Lat: " + std::to_string(latitude_) + ", Long: " + std::to_string(longitude_) + ", Alt: " + std::to_string(altitude_) + "\n";
        statusStr += "Sunrise: " + std::to_string(sunrise_) + "\n";
        statusStr += "Sunset: " + std::to_string(sunset_) + "\n";
        statusStr += "is Daylight: " + std::string(isDaylight_ ? "true" : "false") + "\n";
        statusStr += "is Wet road: " + std::string(isWetRoad_ ? "true" : "false") + "\n";
        statusStr += "is Sky occluded: " + std::string(isSkyOccluded_ ? "true" : "false") + "\n";

        return statusStr;
    }
}
