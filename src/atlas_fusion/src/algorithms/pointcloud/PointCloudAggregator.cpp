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

#include "algorithms/pointcloud/PointCloudAggregator.h"
#include "util/IdentifierToFrameConversions.h"

namespace AutoDrive::Algorithms {

    void PointCloudAggregator::addPointCloudBatches(const std::vector<std::shared_ptr<DataModels::PointCloudBatch>> &batches) {
        //TODO: Too slow
        // Timer t("Add lidar batches");
        size_t pointsToAdd = 0;
        size_t noOfPoints = aggregatedPoints_->size();
        for (const auto &batch: batches) {
            batchInfo_.emplace_back(batch->getTimestamp(), batch->getPointsSize());
            pointsToAdd += batch->getPointsSize();
            pcl::concatenate(*aggregatedPoints_, *batch->getPointsInGlobalCoordinates(), *aggregatedPoints_);
        }
        egoPointsValid_ = false;
        downsampledPointsValid_ = false;
        egoCentricSensorCutouts_.clear();

        assert(noOfPoints + pointsToAdd == aggregatedPoints_->size());
    }


    void PointCloudAggregator::filterOutBatches(uint64_t currentTime) {
        // Timer t("Filter out lidar aggregated batches");
        if (batchInfo_.empty()) return;

        size_t pointsToDelete = 0;
        while (!batchInfo_.empty()) {
            auto timestamp = batchInfo_.front().first;
            auto noOfPoints = batchInfo_.front().second;

            auto timeDiff = static_cast<double>(currentTime - timestamp) * 1e-9;
            if (timeDiff < aggregationTime_) break;

            pointsToDelete += noOfPoints;
            batchInfo_.pop_front();
        }
        if (pointsToDelete == 0) return;
        size_t noOfPoints = aggregatedPoints_->size();
        aggregatedPoints_->points.erase(aggregatedPoints_->points.begin(), aggregatedPoints_->points.begin() + static_cast<long>(pointsToDelete));

        assert(noOfPoints - pointsToDelete == aggregatedPoints_->size());
    }

    pcl::PointCloud<pcl::PointXYZ>::ConstPtr PointCloudAggregator::getGlobalCoordinatePointCloud() {
        // Timer t("Get aggregated point cloud");
        if (downsampledPointsValid_) return aggregatedPointsDownsampled_;

        aggregatedPointsDownsampled_ = pointCloudProcessor_.downsamplePointCloud(aggregatedPoints_);
        downsampledPointsValid_ = true;

        return aggregatedPointsDownsampled_;
    }

    pcl::PointCloud<pcl::PointXYZ>::ConstPtr PointCloudAggregator::getEgoCentricPointCloud(const rtl::RigidTf3D<double> &egoTf) {
        // Timer t("Get ego centric point cloud");

        if (egoPointsValid_) return egoCentricPoints_;

        egoCentricPoints_ = pointCloudProcessor_.transformPointCloud(aggregatedPointsDownsampled_, egoTf);
        //pointCloudProcessor_.sortPointCloud(egoCentricPoints_, PointCloudProcessor::Axis::Z, false);
        egoPointsValid_ = true;

        return egoCentricPoints_;
    }

    pcl::PointCloud<pcl::PointXYZ>::ConstPtr PointCloudAggregator::getSensorPointCloudCutout(const rtl::RigidTf3D<double> &egoTf, const FrameType &frame) {
        //Timer t("Get ego centric point cloud cutout for frame: " + frameTypeName(frame), 0);
        if (!egoPointsValid_) getEgoCentricPointCloud(egoTf);

        std::string f = frameTypeName(frame);
        if (egoCentricSensorCutouts_.count(f) == 0) {
            egoCentricSensorCutouts_[f] = pointCloudProcessor_.getPointCloudCutoutForFrame(aggregatedPointsDownsampled_, frame);
        }

        return egoCentricSensorCutouts_[f];
    }
}