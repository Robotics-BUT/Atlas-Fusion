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

#include <queue>
#include <rtl/Core.h>

#include "Context.h"
#include "data_models/local_map/PointCloudBatch.h"
#include "data_models/local_map/LocalPosition.h"

namespace AtlasFusion::Algorithms {

    /**
     * Point Cloud Aggregator is a class designed to accept point cloud batches on the input, and to keep this batch in
     * the memory for a defined time. It also provides very basic space-point filtration.
     */
    class PointCloudAggregator {

    public:

        /**
         * Constructor
         * @param context container for global services, logging, time stamping, etc.
         * @param aggTime time in secs for how long point will be keeped in the memory
         */
        explicit PointCloudAggregator(Context& context, float aggTime)
        : context_{context}
        , aggregationTime_{aggTime} {

        }

        /**
         * Insert new batches into the aggregation memory
         * @param batches std::vector of new batches that have to be aggregated
         */
        void addPointCloudBatches(std::vector<std::shared_ptr<DataModels::PointCloudBatch>> batches);

        /**
         * Method filters out all the batches that are older than (given time) - (aggregation time from constructor)
         * @param currentTime reference time to remove older batches
         */
        void filterOutBatches(uint64_t currentTime);

        /**
         * Getter for all batches currently aggregated in the class instance.
         * @return vector of shared pointers on aggregated batches
         */
        std::vector<std::shared_ptr<DataModels::PointCloudBatch>> getAllBatches();

        /**
         * Getter for all point clouds in global coordinate system represented by the aggregated batches.
         * @return point cloud in global coordinate system
         */
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> getAggregatedPointCloud();

        /**
         * Method filters out all the points that are out of given bounding box.
         * @param input Input point cloud.
         * @param borders Points that are inside this bounding box will be passed to the output point cloud.
         * @return Point cloud that contains points from inside of the given bounding box.
         */
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> getPointcloudCutout(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> input, rtl::BoundingBox3f borders);

    private:

        Context& context_;

        double aggregationTime_;
        std::deque<std::shared_ptr<DataModels::PointCloudBatch>> batchQueue_;

    };
}
