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

#include "data_models/lidar/LidarScanDataModel.h"
#include "data_models/local_map/PointCloudBatch.h"
#include "data_models/local_map/LocalPosition.h"

namespace AtlasFusion::Algorithms {

    /**
     * Point Cloud Extrapolator takes point cloud scan on the input and based on the given poisition in which the the
     * scanner was at the very beginning of the scanning and at very end in splits the scan into N batches and linearly
     * interpolates points's poisition. In this way the lidar-motion undistortion is done.
     */
    class PointCloudExtrapolator {

    public:

        /**
         * Contructor
         * @param context global services container, like time, TF tree, etc.
         * @param noOfBatcher number of batches the scan will be splitted into
         */
        explicit PointCloudExtrapolator(Context& context, size_t noOfBatcher)
        : context_{context}
        , noOfBatches_{noOfBatcher} {

        }

        /**
         * Process single scan and performs lidar-motion undistortion
         * @param scan input lidar scan
         * @param startPose car position at the moment the scan started
         * @param poseDiff car position at the end of scan
         * @param sensorOffset sensor to car offset
         * @return Returns the vector of shared pointers on batches that represents the undistorted input scan
         */
        std::vector<std::shared_ptr<DataModels::PointCloudBatch>> splitPointCloudToBatches(
                std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> scan,
                DataModels::LocalPosition startPose,
                DataModels::LocalPosition poseDiff,
                rtl::RigidTf3D<double> sensorOffset);

        /**
         * Setter for internal variable that defines number of batches that the input scan will be splitted into.
         * @param n nubmer of batches
         */
        void setNoOfBatches(size_t n) {noOfBatches_ = n;};

    private:

        Context& context_;
        size_t noOfBatches_;

        DataModels::LocalPosition interpolateLocalPosition(
                DataModels::LocalPosition& begin,
                DataModels::LocalPosition& end,
                float ratio);

    };

}
