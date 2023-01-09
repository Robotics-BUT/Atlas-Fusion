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
        //TODO: This is heavy operation when done on the whole aggregated point cloud
        // Timer t("Downsampling points");
        pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::VoxelGrid<pcl::PointXYZ> downsampler;
        downsampler.setInputCloud(input);
        downsampler.setLeafSize(leafSize_, leafSize_, leafSize_);
        downsampler.filter(*output);

        return output;
    }

    void PointCloudProcessor::downsamplePointCloudInPlace(pcl::PointCloud<pcl::PointXYZ>::Ptr &input) {

        pcl::VoxelGrid<pcl::PointXYZ> downsampler;
        downsampler.setInputCloud(input);
        downsampler.setLeafSize(leafSize_, leafSize_, leafSize_);
        downsampler.filter(*input);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr
    PointCloudProcessor::transformPointCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &input, const rtl::RigidTf3D<double> &tf) {
        //TODO: This function is really slow concatenating point clouds doesn't really have an alternative that I know of.
        //Timer t("Transform point cloud");
        pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
        if (input->points.empty()) return output;
        output->reserve(input->size());

        auto threads = context_.threadPool_.get_thread_count();
        std::vector<std::future<pcl::PointCloud<pcl::PointXYZ>::Ptr>> outputFutures;
        outputFutures.resize(threads);
        size_t batchSize = std::ceil(input->points.size() / threads);

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
        pcl_tf.translation() << float(tf.trVecX()), float(tf.trVecY()), float(tf.trVecZ());

        for (uint32_t i = 0; i < threads; i++) {
            outputFutures[i] = context_.threadPool_.submit([&input, pcl_tf, i, batchSize, threads] {
                pcl::PointCloud<pcl::PointXYZ>::Ptr outBatch(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::PointCloud<pcl::PointXYZ>::Ptr inBatch(new pcl::PointCloud<pcl::PointXYZ>);

                // Filter selected part of the input point cloud
                uint32_t start = i * batchSize;
                uint32_t end = start + batchSize;
                if (i == threads - 1) end = input->points.size();

                inBatch->reserve(end - start);
                outBatch->width = end - start;

                std::copy(input->begin() + start, input->begin() + end, back_inserter(inBatch->points));

                pcl::transformPointCloud(*inBatch, *outBatch, pcl_tf);
                return outBatch;
            });
        }

        for (auto &outputFuture: outputFutures) {
            outputFuture.wait();
            pcl::concatenate(*output, *outputFuture.get(), *output);
        }

        assert(input->points.size() == output->points.size());

        return output;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr
    PointCloudProcessor::getAggregatedAboveGroundPoints(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &input) {
        Timer t("Get point cloud cutout");

        pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
        output->reserve(input->size());

        for (size_t p = 0; p < input->size(); p++) {
            const auto &point = input->points.at(p);
            // We can prematurely return because the points are ordered in Z axis
            if (point.z < -0.75) {
                break;
            }
            output->points.emplace_back(point);
        }

        std::cout << "Cutout returns " << std::to_string(output->size()) << " points" << std::endl;
        return output;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr
    PointCloudProcessor::getPointCloudCutout(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &input, const rtl::BoundingBox3f &boundingBox) {
        // TODO: Very slow -> Find a way to make those cutouts without iterating over the whole pc.
        Timer t("Get point cloud cutout");

        pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
        output->reserve(input->size());

        for (const auto &point: input->points) {
            if (point.x > boundingBox.min().getElement(0) && point.x < boundingBox.max().getElement(0) &&
                point.y > boundingBox.min().getElement(1) && point.y < boundingBox.max().getElement(1) &&
                point.z > boundingBox.min().getElement(2) && point.z < boundingBox.max().getElement(2)) {
                output->push_back(point);
            }
        }
        return output;
    }

    void PointCloudProcessor::sortPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &input, const Axis &axis, bool ascending) {
        Timer t("Sorting point cloud of length: " + std::to_string(input->width));

        switch (axis) {
            case X:
                std::stable_sort(input->points.begin(), input->points.end(), ascending ? &compareXAsc : &compareXDesc);
                break;
            case Y:
                std::stable_sort(input->points.begin(), input->points.end(), ascending ? &compareYAsc : &compareYDesc);
                break;
            case Z:
                std::stable_sort(input->points.begin(), input->points.end(), ascending ? &compareZAsc : &compareZDesc);
                break;
        }
    }

    bool PointCloudProcessor::compareXAsc(const pcl::PointXYZ &l, const pcl::PointXYZ &r) {
        return l.x < r.x;
    }

    bool PointCloudProcessor::compareYAsc(const pcl::PointXYZ &l, const pcl::PointXYZ &r) {
        return l.y < r.y;
    }

    bool PointCloudProcessor::compareZAsc(const pcl::PointXYZ &l, const pcl::PointXYZ &r) {
        return l.z < r.z;
    }

    bool PointCloudProcessor::compareXDesc(const pcl::PointXYZ &l, const pcl::PointXYZ &r) {
        return l.x > r.x;
    }

    bool PointCloudProcessor::compareYDesc(const pcl::PointXYZ &l, const pcl::PointXYZ &r) {
        return l.y > r.y;
    }

    bool PointCloudProcessor::compareZDesc(const pcl::PointXYZ &l, const pcl::PointXYZ &r) {
        return l.z > r.z;
    }
}