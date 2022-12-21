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

#include <pcl/filters/extract_indices.h>
#include "algorithms/pointcloud/PointCloudExtrapolator.h"
#include "local_map/Frames.h"
#include "Timer.h"

namespace AutoDrive::Algorithms {

    std::vector<std::shared_ptr<DataModels::PointCloudBatch>> PointCloudExtrapolator::splitPointCloudToBatches(
            const pcl::PointCloud<pcl::PointXYZ>::Ptr &scan,
            const DataModels::LocalPosition &startPose,
            const DataModels::LocalPosition &poseDiff,
            const rtl::RigidTf3D<double> &sensorOffsetTf) {
        Timer t("Split point cloud into batches");

        std::vector<std::shared_ptr<DataModels::PointCloudBatch>> output;
        output.resize(noOfBatches_);

        uint32_t singleBatchSize = std::ceil(scan->width / noOfBatches_);
        uint64_t timeOffset = startPose.getTimestamp();
        uint64_t duration = poseDiff.getTimestamp();
        DataModels::LocalPosition endPose{startPose.getPosition() + poseDiff.getPosition(),
                                          startPose.getOrientation() * poseDiff.getOrientation(),
                                          startPose.getTimestamp() + poseDiff.getTimestamp()};

        std::vector<std::future<std::shared_ptr<DataModels::PointCloudBatch>>> outputFutures;
        outputFutures.resize(noOfBatches_);

        for (uint32_t index = 0; index < noOfBatches_; index++) {

            outputFutures.at(index) = context_.threadPool.submit(
                    [=]() mutable {
                        double ratio = (double) index / noOfBatches_;
                        auto pose = AutoDrive::DataModels::LocalPosition{
                                {poseDiff.getPosition().x() * (ratio), poseDiff.getPosition().y() * (ratio), poseDiff.getPosition().z() * (ratio)},
                                {poseDiff.getOrientation().slerp(rtl::Quaternion<double>::identity(), (float) (1 - ratio))},
                                uint64_t(duration * ratio)
                        };

                        rtl::RigidTf3D<double> movementCompensationTF{pose.getOrientation(), pose.getPosition()};
                        uint64_t ts = timeOffset + static_cast<uint64_t>(ratio * duration);

                        pcl::PointCloud<pcl::PointXYZ> points;
                        points.reserve(singleBatchSize);

                        pcl::Indices indices(singleBatchSize);
                        std::iota(indices.begin(), indices.end(), index * singleBatchSize);

                        pcl::ExtractIndices<pcl::PointXYZ> eifilter(false);
                        eifilter.setInputCloud(scan);
                        eifilter.setIndices(boost::make_shared<pcl::Indices>(indices));
                        eifilter.filter(points);

                        rtl::RigidTf3D<double> toGlobalFrameTf{endPose.getOrientation(), endPose.getPosition()};
                        rtl::RigidTf3D<double> startToEndTf{poseDiff.getOrientation(), poseDiff.getPosition()};
                        auto finalTF = toGlobalFrameTf(startToEndTf.inverted()(movementCompensationTF(sensorOffsetTf)));

                        return std::make_shared<DataModels::PointCloudBatch>(ts, points.makeShared(), LocalMap::Frames::kOrigin, finalTF);
                    });
        }
        context_.threadPool.wait_for_tasks();

        for (uint32_t index = 0; index < noOfBatches_; index++) {
            output.at(index) = outputFutures.at(index).get();
        }

        return output;
    }
}
