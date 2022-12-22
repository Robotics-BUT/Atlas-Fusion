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

#include "algorithms/pointcloud/PointCloudProcessor.h"
#include "Timer.h"

#include <pcl/common/transforms.h>

namespace AutoDrive::Algorithms {


    pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudProcessor::downsamplePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input) {

        pcl::PointCloud<pcl::PointXYZ> output;

        pcl::VoxelGrid<pcl::PointXYZ> downsampler;
        downsampler.setInputCloud(input);
        downsampler.setLeafSize(leafSize_, leafSize_, leafSize_);
        downsampler.filter(output);

        return output.makeShared();
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudProcessor::transformPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input, const rtl::RigidTf3D<double> &tf) {

        pcl::PointCloud<pcl::PointXYZ>::Ptr output{};
        output->reserve(input->size());

        auto rotMat = tf.rotMat();
        Eigen::Affine3f pcl_tf = Eigen::Affine3f::Identity();
        pcl_tf(0, 0) = static_cast<float>(rotMat(0, 0));
        pcl_tf(1, 0) = static_cast<float>(rotMat(1, 0));
        pcl_tf(2, 0) = static_cast<float>(rotMat(2, 0));
        pcl_tf(0, 1) = static_cast<float>(rotMat(0, 1));
        pcl_tf(1, 1) = static_cast<float>(rotMat(1, 1));
        pcl_tf(2, 1) = static_cast<float>(rotMat(2, 1));
        pcl_tf(0, 2) = static_cast<float>(rotMat(0, 2));
        pcl_tf(1, 2) = static_cast<float>(rotMat(1, 2));
        pcl_tf(2, 2) = static_cast<float>(rotMat(2, 2));
        pcl_tf.translation() << tf.trVecX(), tf.trVecY(), tf.trVecZ();
        pcl::transformPointCloud(*input, *output, pcl_tf);

        return output;
    }
}