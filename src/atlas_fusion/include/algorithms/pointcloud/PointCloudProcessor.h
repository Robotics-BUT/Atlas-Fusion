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

namespace AtlasFusion::Algorithms {

    /**
     * Point Cloud Processor encapsulates simple operations on point clouds, like downsampling or applying the
     * transformation on a point cloud.
     */

    class PointCloudProcessor {

    public:

        /**
         * Constructor
         * @param context global services container (time stamps provider, TF tree, logger, etc.)
         * @param leafSize the leaf size of the downsampled point cloud
         */
        PointCloudProcessor(Context& context, float leafSize)
        : context_{context}
        , leafSize_{leafSize} {

        }

        /**
         * Method downsamples the point cloud given at the input
         * @param input given point cloud that will be downsampled
         * @return downsampled point cloud
         */
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> downsamplePointCloud(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> input);

        /**
         * Method applies 3D transpormation on a given point cloud
         * @param input input point cloud
         * @param tf 3D transformation that will be applied
         * @return transpormed point cloud
         */
        static std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> transformPointcloud(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> input, rtl::RigidTf3D<double> tf);

    private:

        Context& context_;

        float leafSize_;
    };

}