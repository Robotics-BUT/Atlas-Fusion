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
#include <pcl/filters/voxel_grid.h>

#include "Context.h"

namespace AutoDrive::Algorithms {

    /**
     * Point Cloud Processor encapsulates simple operations on point clouds, like downsampling or applying the
     * transformation on a point cloud.
     */

    class PointCloudProcessor {

    public:

        enum Axis {
            X, Y, Z
        };

        /**
         * Constructor
         * @param context global services container (time stamps provider, TF tree, logger, etc.)
         * @param leafSize the leaf size of the downsampled point cloud
         */
        PointCloudProcessor(Context &context, float leafSize)
                : context_{context}, leafSize_{leafSize} {

        }

        /**
         * Method downsamples the point cloud given at the input
         * @param input given point cloud that will be downsampled
         * @return downsampled point cloud
         */
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsamplePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input);

        /**
        * Method downsamples in place the point cloud given at the input
        * @param input given point cloud that will be downsampled
        */
        void downsamplePointCloudInPlace(pcl::PointCloud<pcl::PointXYZ>::Ptr &input);


        /**
         * Method applies 3D transformation on a given point cloud
         * @param input input point cloud
         * @param tf 3D transformation that will be applied
         * @return transformed point cloud
         */
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformPointCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &input, const rtl::RigidTf3D<double> &tf);

        /**
         * This function presumes that the input aggregated point cloud has batches ordered by Z axis in the ascending order, failing to pass such will not work properly
         * @param input input aggregate point cloud
         * @param batchLengths lengths of individual batches inside the aggregated point cloud
         * @param boundingBox 3D bound inside which the points will be returned
         * @return point cloud cutout
         */
        pcl::PointCloud<pcl::PointXYZ>::Ptr
        getAggregatedGroundPoints(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &input, const std::vector<long> &batchLengths);
        /**
         *
         * @param input input point cloud
         * @param boundingBox 3D bound inside which the points will be returned
         * @return point cloud cutout
         */
        pcl::PointCloud<pcl::PointXYZ>::Ptr getPointCloudCutout(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &input, const rtl::BoundingBox3f &boundingBox);


        /**
         *
         * @param input input point cloud
         * @param axis dimension by which the point cloud is going to be sorted
         */
        void sortPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &input, const Axis &axis, bool ascending = true);

    private:

        Context &context_;
        float leafSize_;

        static bool compareX(const pcl::PointXYZ &l, const pcl::PointXYZ &r);
        static bool compareY(const pcl::PointXYZ &l, const pcl::PointXYZ &r);
        static bool compareZ(const pcl::PointXYZ &l, const pcl::PointXYZ &r);
    };

}