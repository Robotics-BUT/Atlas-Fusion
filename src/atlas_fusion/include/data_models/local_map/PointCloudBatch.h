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

namespace AtlasFusion::DataModels {

    /**
     * Point Cloud Batch represents small part of the LiDAR scan. Splitting scans into batches is used during the
     * motion undistorting process. Batch holds the points and the transformation. This transformation is lazy evaluated
     * so the is no redundant transformation application on all points. In time transformation can be chained and
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
        explicit PointCloudBatch(uint64_t ts, pcl::PointCloud<pcl::PointXYZ> points, std::string frame, rtl::RigidTf3D<double> tf)
        : timestamp_{ts}
        , points_{std::move(points)}
        , referenceFrame_{std::move(frame)}
        , tf_{tf} {

        }

        /**
         * Untransformed points getter
         * @return untransformed points
         */
        std::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>> getPoints() const;

        /**
         * Points with applied transformation getter
         * @return transformed 3D points
         */
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> getTransformedPoints() const;

        /**
         * Double transformed points ( points transformed by this->tf transformed by argument tf)
         * @param tf second transformation applied on points
         * @return transformed points
         */
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> getTransformedPointsWithAnotherTF(rtl::RigidTf3D<double>& tf) const;

        /**
         * Timestamp when the points have been scanned
         * @return nanosecond timestamp
         */
        uint64_t getTimestamp() const;

        /**
         * Method returns number of points in the batch
         * @return batch size
         */
        size_t getPointsSize() const;

        /**
         * Method returns frame of the LiDAR that has scanned points
         * @return string name of the sensnor's frame
         */
        std::string getFrame() const;

        /**
         * Points transformation getter
         * @return transformtaion that should be applied on points to get the real 3D position
         */
        rtl::RigidTf3D<double> getTF() const;

    private:
        uint64_t timestamp_;
        pcl::PointCloud<pcl::PointXYZ> points_;
        std::string referenceFrame_;
        rtl::RigidTf3D<double> tf_;

        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> transformPointsByTF( const rtl::RigidTf3D<double>& tf, const pcl::PointCloud<pcl::PointXYZ>& pts ) const;
    };

}