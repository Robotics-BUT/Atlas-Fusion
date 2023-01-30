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

#include "fail_check/LidarFailChecker.h"
#include "util/IdentifierToFrameConversions.h"

namespace AutoDrive::FailCheck {


    void LidarFailChecker::onNewData(const std::shared_ptr<DataModels::LidarScanDataModel> &data) {
        Timer t("Lidar fail checker");
        auto scan = data->getScan();
        frameType_ = frameTypeFromDataModel(data);
        pointCount_ = data->getScan()->size();

        if (frameType_ == FrameType::kLidarCenter) {
            auto road = pointCloudProcessor_.getPointCloudCutout(scan, {{0.0f, -2.0f, -0.75f}, {15.0f, 2.0f, -2.0f}});
            roiPointCount_ = road->size();
        }

        pointCloudProcessor_.sortPointCloudByDistance(scan, false);
        auto &p = scan->front();
        range_ = range_ * 0.99 + 0.01 *std::sqrt(std::pow(p.x, 2) + std::pow(p.y, 2) + std::pow(p.z, 2));

        evaluatePerformance();
    }

    void LidarFailChecker::evaluatePerformance() {
        if (frameType_ == FrameType::kLidarCenter) {
            isWetRoad_ = roiPointCount_ < 2500;
            environmentalModel_.setIsWetRoad(isWetRoad_);
        }

        sensorStatus_.statusString = frameTypeName(frameType_) + "\n";

        sensorStatus_.statusMap["scan_points"] = pointCount_;
        sensorStatus_.statusString += "Scan points: " + std::to_string(pointCount_) + "\n";

        sensorStatus_.statusMap["range"] = range_;
        sensorStatus_.statusString += "Range: " + std::to_string(range_) + + " m" " \n";

        if (frameType_ == FrameType::kLidarCenter) {
            sensorStatus_.statusMap["roi_points"] = roiPointCount_;
            sensorStatus_.statusString += "ROI points: " + std::to_string(roiPointCount_) + "\n";

            sensorStatus_.statusMap["possible_wet_road"] = isWetRoad_;
            sensorStatus_.statusString += "Possible wet road: " + std::to_string(isWetRoad_) + "\n";
        }
    }
}