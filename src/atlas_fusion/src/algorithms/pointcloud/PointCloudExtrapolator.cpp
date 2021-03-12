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

#include "algorithms/pointcloud/PointCloudExtrapolator.h"
#include "local_map/Frames.h"

namespace AtlasFusion::Algorithms {

    std::vector<std::shared_ptr<DataModels::PointCloudBatch>> PointCloudExtrapolator::splitPointCloudToBatches(
            std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> scan,
            DataModels::LocalPosition startPose,
            DataModels::LocalPosition poseDiff,
            rtl::RigidTf3D<double> sensorOffsetTf) {


        std::vector<std::shared_ptr<DataModels::PointCloudBatch>> output;

        auto singleBatchSize = static_cast<size_t>(std::ceil(static_cast<float>(scan->width) / noOfBatches_));
        auto timeOffset = startPose.getTimestamp();
        auto duration = poseDiff.getTimestamp();
        DataModels::LocalPosition endPose {startPose.getPosition() + poseDiff.getPosition(),
                                           startPose.getOrientation() * poseDiff.getOrientation(),
                                           startPose.getTimestamp() + poseDiff.getTimestamp()};

        size_t pointCnt = 0;
        for(size_t i = 0 ; i < noOfBatches_ ; i++) {

            double ratio = static_cast<double>(i) / noOfBatches_;
            auto pose = AtlasFusion::DataModels::LocalPosition {
                    {poseDiff.getPosition().x() * (ratio), poseDiff.getPosition().y() * (ratio), poseDiff.getPosition().z() * (ratio)},
                    {poseDiff.getOrientation().slerp(rtl::Quaternion<double>::identity(), (float)(1-ratio))},
                    uint64_t(duration * (ratio))
            };

            rtl::RigidTf3D<double> movementCompensationTF{pose.getOrientation(), pose.getPosition()};
            uint64_t ts = timeOffset + static_cast<uint64_t>(ratio * duration);

            pcl::PointCloud<pcl::PointXYZ> points;
            points.reserve(singleBatchSize);

            // TODO: Avoid point copying
            for(size_t j = 0 ; j < singleBatchSize ; j++) {
                if(pointCnt < scan->width) {
                    points.push_back(scan->at(pointCnt++));
                }
            }

            rtl::RigidTf3D<double> toGlobelFrameTf{endPose.getOrientation(), endPose.getPosition()};
            rtl::RigidTf3D<double> startToEndTf{poseDiff.getOrientation(), poseDiff.getPosition()};
            auto finalTF = toGlobelFrameTf(startToEndTf.inverted()(movementCompensationTF(sensorOffsetTf)));

            // TODO: Avoid point copying
            auto batch = std::make_shared<DataModels::PointCloudBatch>(ts, points, LocalMap::Frames::kOrigin, finalTF);
            output.push_back(batch);
        }

        return output;
    }
}
