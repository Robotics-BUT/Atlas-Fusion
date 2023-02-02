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
    /** Lidar scan aggregation section */

    void PointCloudAggregator::addLidarScan(const DataLoader::LidarIdentifier &lidarIdentifier,
                                            const std::vector<std::shared_ptr<DataModels::PointCloudBatch>> &scanBatches) {
        Timer t("Add lidar scan");
        switch (lidarIdentifier) {
            case DataLoader::LidarIdentifier::kLeftLidar: {
                lidarLeftBatches_ = scanBatches;
                break;
            }
            case DataLoader::LidarIdentifier::kCenterLidar: {
                lidarCenterBatches_ = scanBatches;
                break;
            }
            case DataLoader::LidarIdentifier::kRightLidar: {
                lidarRightBatches_ = scanBatches;
                break;
            }
            case DataLoader::LidarIdentifier::kNone:
                break;
        }
        latestScanCutouts_.clear();
        latestScanValid_ = false;
        latestScanEgoValid_ = false;
    }

    pcl::PointCloud<pcl::PointXYZ>::ConstPtr PointCloudAggregator::getLatestScan() {
        Timer t("Get latest scan");

        if (latestScanValid_) return latestScanPoints_;

        pcl::PointCloud<pcl::PointXYZ>::Ptr scan(new pcl::PointCloud<pcl::PointXYZ>);
        scan->height = 1;
        if (!lidarLeftBatches_.empty()) {
            auto left = getPointCloudFromBatches(lidarLeftBatches_);
            pcl::concatenate(*scan, *left, *scan);
        }
        if (!lidarCenterBatches_.empty()) {
            auto center = getPointCloudFromBatches(lidarCenterBatches_);
            pcl::concatenate(*scan, *center, *scan);
        }
        if (!lidarRightBatches_.empty()) {
            auto right = getPointCloudFromBatches(lidarRightBatches_);
            pcl::concatenate(*scan, *right, *scan);
        }
        scan->width = scan->points.size();
        latestScanPoints_ = scan->makeShared();
        latestScanValid_ = true;

        return latestScanPoints_;
    }

    pcl::PointCloud<pcl::PointXYZ>::ConstPtr
    PointCloudAggregator::getLatestScanEgoCentric(const rtl::RigidTf3D<double> &egoTf) {
        // Timer t("Get ego centric point cloud");

        if (latestScanEgoValid_) return latestScanEgoPoints_;

        latestScanEgoPoints_ = pointCloudProcessor_.transformPointCloud(getLatestScan(), egoTf);
        latestScanEgoValid_ = true;

        return latestScanEgoPoints_;
    }


    pcl::PointCloud<pcl::PointXYZ>::ConstPtr
    PointCloudAggregator::getLatestScanCutout(const rtl::RigidTf3D<double> &egoTf, const FrameType &frame) {
        //Timer t("Get latest scan cutout for frame: " + frameTypeName(frame), 0);
        if (!latestScanValid_) getLatestScanEgoCentric(egoTf);

        std::string f = frameTypeName(frame);
        if (latestScanCutouts_.count(f) == 0) {
            latestScanCutouts_[f] = pointCloudProcessor_.getPointCloudCutoutForFrame(latestScanEgoPoints_, frame);
        }

        return latestScanCutouts_[f];
    }


    /** Short term aggregation section */

    void PointCloudAggregator::addPointCloudBatches(
            const std::vector<std::shared_ptr<DataModels::PointCloudBatch>> &batches) {
        Timer t("Add lidar batches");

        auto downsampledBatches = std::vector<std::shared_ptr<DataModels::PointCloudBatch>>();
        for (const auto &batch: batches) {
            auto points = pointCloudProcessor_.downsamplePointCloud(batch->getPoints());
            downsampledBatches.emplace_back(
                    std::make_shared<DataModels::PointCloudBatch>(
                            DataModels::PointCloudBatch(pointCloudProcessor_, batch->getTimestamp(), points,
                                                        batch->getFrame(),
                                                        batch->getGlobalTf()))
            );
            batchInfo_.emplace_back(batch->getTimestamp(), points->size());
        }

        auto points = getPointCloudFromBatches(downsampledBatches);
        pcl::concatenate(*aggregatedPoints_, *points, *aggregatedPoints_);

        aggregatedDownsampledPointsValid_ = false;
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
        if (pointsToDelete > aggregatedPoints_->size()) {
            aggregatedPoints_->points.clear();
            return;
        }

        aggregatedPoints_->points.erase(aggregatedPoints_->points.begin(),
                                        aggregatedPoints_->points.begin() + static_cast<long>(pointsToDelete));
    }

    pcl::PointCloud<pcl::PointXYZ>::ConstPtr PointCloudAggregator::getGlobalCoordinateAggregatedPointCloud() {
        Timer t("Get aggregated point cloud");
        if (aggregatedDownsampledPointsValid_) return aggregatedPointsDownsampled_;

        aggregatedPointsDownsampled_ = pointCloudProcessor_.downsamplePointCloud(aggregatedPoints_);
        aggregatedDownsampledPointsValid_ = true;

        return aggregatedPointsDownsampled_;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr
    PointCloudAggregator::getPointCloudFromBatches(
            const std::vector<std::shared_ptr<DataModels::PointCloudBatch>> &batches) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);

        uint32_t totalPoints = 0;
        for (const auto &batch: batches) {

            auto batchPc = batch->getPointsInGlobalCoordinates();
            totalPoints += batchPc->width;
            pcl::concatenate(*pc, *batchPc, *pc);
        }

        if (totalPoints != pc->size()) {
            throw std::runtime_error("Mismatch during batches to point cloud conversion!");
        }

        return pc;
    }
}