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

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include <rtl/Transformation.h>

#include <utility>
#include "algorithms/pointcloud/PointCloudProcessor.h"

namespace AutoDrive::DataModels {

    /**
     * Point Cloud Batch represents small part of the LiDAR scan. Splitting scans into batches is used during the
     * motion undistorting process. Batch holds the points and the transformation. This transformation is lazy evaluated
     * so there is no redundant transformation application on all points. In time transformation can be chained and
     * could be applied only once when the points are needed.
     */
    class PointCloudBatch {

    public:

        PointCloudBatch() = delete;

        /**
         * Constructor
         * @param ts timestamp, when the points have been measured
         * @param untransformed points 3D points
         * @param frame sensor's frame
         * @param tf transformation that should be applied on points to get real position in 3D space
         */
        explicit PointCloudBatch(Algorithms::PointCloudProcessor &pcProcessor, uint64_t ts, pcl::PointCloud<pcl::PointXYZ>::Ptr points, std::string frame,
                                 const rtl::RigidTf3D<double> &tf)
                : pointCloudProcessor_{pcProcessor}, timestamp_{ts}, points_{std::move(points)}, referenceFrame_{std::move(frame)}, tf_{tf} {}

        /**
         * Untransformed points getter
         * @return untransformed points
         */
        [[nodiscard]] pcl::PointCloud<pcl::PointXYZ>::Ptr getPoints() const;

        /**
         * Points with applied transformation
         * @return transformed 3D points
         */
        [[nodiscard]] pcl::PointCloud<pcl::PointXYZ>::Ptr getTransformedPoints() const;

        /**
         * Double transformed points ( points transformed by this->tf transformed by argument tf)
         * @param tf second transformation applied on points
         * @return transformed points
         */
        pcl::PointCloud<pcl::PointXYZ>::Ptr getTransformedPointsWithAnotherTF(rtl::RigidTf3D<double> &tf) const;

        /**
         * Timestamp when the points have been scanned
         * @return nanosecond timestamp
         */
        [[nodiscard]] uint64_t getTimestamp() const { return timestamp_; };


        /**
         * Method returns number of points in the batch
         * @return batch size
         */
        [[nodiscard]] size_t getPointsSize() const { return points_->size(); };

        /**
         * Method returns frame of the LiDAR that has scanned points
         * @return string name of the sensnor's frame
         */
        [[nodiscard]] std::string getFrame() const { return referenceFrame_; };

        /**
         * Points transformation getter
         * @return transformtaion that should be applied on points to get the real 3D position
         */
        [[nodiscard]] rtl::RigidTf3D<double> getTF() const { return tf_; };

    private:
        Algorithms::PointCloudProcessor &pointCloudProcessor_;

        uint64_t timestamp_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr points_;
        std::string referenceFrame_;
        rtl::RigidTf3D<double> tf_;
    };

}