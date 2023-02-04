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

#include "data_models/local_map/PointCloudBatch.h"
#include "data_models/local_map/LocalPosition.h"
#include "data_models/lidar/LidarScanDataModel.h"

namespace AutoDrive::Algorithms {

    /**
     * Point Cloud Aggregator a class designed to aggregate LiDAR scans so we can work with them together.
     * There are two ways to use this aggregator. Either by only holding one scan per lidar at a time (used for all algorithms down the pipe),
     * or by aggregating all scans for a given time period (For short term map building [only for visualization]).
     * It also provides very basic space-point transformation / filtration.
     */
    class PointCloudAggregator {

    public:

        /**
         * Constructor
         * @param context container for global services, logging, time stamping, etc.
         * @param aggTime time in secs for how long point will be kept in the memory
         */
        explicit PointCloudAggregator(Context &context, PointCloudProcessor &pcProcessor, float aggTime)
                : context_{context}, pointCloudProcessor_{pcProcessor}, aggregationTime_{aggTime},
                  aggregatedPoints_{new pcl::PointCloud<pcl::PointXYZ>} {}

        /** Lidar scan aggregation section */

        /**
        * Insert new lidar scan
        * @param scan raw lidar scan data model
        */
        void addLidarScan(const DataLoader::LidarIdentifier &lidarIdentifier,
                          const std::vector<std::shared_ptr<DataModels::PointCloudBatch>> &scanBatches);

        /**
        * Getter for all point clouds in global coordinate system.
        * @return point cloud in global coordinate system
        */
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr getLatestScan();

        /**
        * Getter for all point clouds relative to the ego vehicle.
        * @return point cloud in ego centric coordinate system
        */
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr getLatestScanEgoCentric(const rtl::RigidTf3D<double> &egoTf);

        /**
        * Get all points in sensor frame FOV
        * @return point cloud cutout in ego centric coordinate system
        */
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr getLatestScanCutout(const rtl::RigidTf3D<double> &egoTf, const FrameType &frame);





        /** Short term aggregation section */

        /**
         * Insert new batches into the aggregation memory
         * @param batches std::vector of new batches that have to be aggregated
         */
        void addPointCloudBatches(const std::vector<std::shared_ptr<DataModels::PointCloudBatch>> &batches);

        /**
         * Method filters out all the batches that are older than (given time) - (aggregation time from constructor)
         * @param currentTime reference time to remove older batches
         */
        void filterOutBatches(uint64_t currentTime);

        /**
         * Getter for all point clouds in global coordinate system.
         * @return point cloud in global coordinate system
         */
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr getGlobalCoordinateAggregatedPointCloud();

    private:

        Context &context_;
        PointCloudProcessor &pointCloudProcessor_;

        pcl::PointCloud<pcl::PointXYZ>::Ptr
        getPointCloudFromBatches(const std::vector<std::shared_ptr<DataModels::PointCloudBatch>> &batches);

        /** Lidar scan aggregation section */
        std::vector<std::shared_ptr<DataModels::PointCloudBatch>> lidarLeftBatches_{}, lidarCenterBatches_{}, lidarRightBatches_{};
        pcl::PointCloud<pcl::PointXYZ>::Ptr latestScanPoints_{}, latestScanEgoPoints_{};

        std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> latestScanCutouts_{};

        bool latestScanValid_ = false;
        bool latestScanEgoValid_ = false;

        /** Short term aggregation section */
        double aggregationTime_;

        pcl::PointCloud<pcl::PointXYZ>::Ptr aggregatedPoints_{};
        pcl::PointCloud<pcl::PointXYZ>::Ptr aggregatedPointsDownsampled_{};

        bool aggregatedDownsampledPointsValid_ = false;

        // Holds timestamp (first) and number of points (second) of all batches added in order
        std::deque<std::pair<uint64_t, long>> batchInfo_;
    };
}
