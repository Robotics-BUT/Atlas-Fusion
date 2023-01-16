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

namespace AutoDrive::Algorithms {

    /**
     * Point Cloud Aggregator is a class designed to accept point cloud batches on the input, and to keep this batch in
     * the memory for a defined time. It also provides very basic space-point filtration.
     */
    class PointCloudAggregator {

    public:

        /**
         * Constructor
         * @param context container for global services, logging, time stamping, etc.
         * @param aggTime time in secs for how long point will be kept in the memory
         */
        explicit PointCloudAggregator(Context &context, PointCloudProcessor &pcProcessor, float aggTime)
                : context_{context}, pointCloudProcessor_{pcProcessor}, aggregationTime_{aggTime}, aggregatedPoints_{new pcl::PointCloud<pcl::PointXYZ>} {}

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
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr getGlobalCoordinatePointCloud();

        /**
        * Getter for all point clouds relative to the ego vehicle.
        * @return point cloud in global coordinate system
        */
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr getEgoCentricPointCloud(const rtl::RigidTf3D<double> &egoTf);

    private:

        Context &context_;
        PointCloudProcessor &pointCloudProcessor_;

        double aggregationTime_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr aggregatedPoints_{};
        pcl::PointCloud<pcl::PointXYZ>::Ptr aggregatedPointsDownsampled_{};

        pcl::PointCloud<pcl::PointXYZ>::Ptr egoCentricPoints_{};

        bool downsampledPointsValid_ = false;
        bool egoPointsValid_ = false;

        // Holds timestamp (first) and number of points (second) of all batches added in order
        std::deque<std::pair<uint64_t, long>> batchInfo_;
    };
}
