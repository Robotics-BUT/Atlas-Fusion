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

#include "algorithms/pointcloud/LaserAggregator.h"

namespace AtlasFusion::Algorithms {

    void LaserAggregator::onNewLaserData(
            std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> scan,
            DataModels::LocalPosition startPose,
            DataModels::LocalPosition poseDiff,
            rtl::RigidTf3D<double> sensorOffsetTf) {

        auto duration = poseDiff.getTimestamp();
        DataModels::LocalPosition endPose {startPose.getPosition() + poseDiff.getPosition(),
                                           startPose.getOrientation() * poseDiff.getOrientation(),
                                           startPose.getTimestamp() + poseDiff.getTimestamp()};

        if((scan->width * scan->height) % noOfLasers_ != 0) {
            context_.logger_.warning(" [Points in lidar scan] modulo [no of lasers] is not zero");
            return;
        }

        auto pointsPerLaser = (scan->width * scan->height) / noOfLasers_;

        for(size_t i = 0 ; i < pointsPerLaser ; i++) {

            double ratio = (float)i / (float)pointsPerLaser;

            auto pose = AtlasFusion::DataModels::LocalPosition {
                    {poseDiff.getPosition().x() * (ratio), poseDiff.getPosition().y() * (ratio), poseDiff.getPosition().z() * (ratio)},
                    {poseDiff.getOrientation().slerp(rtl::Quaternion<double>::identity(), (float)(1-ratio))},
                    uint64_t(duration * (ratio))
            };

            rtl::RigidTf3D<double> globalTF{endPose.getOrientation(), endPose.getPosition()};
            rtl::RigidTf3D<double> startToEndTf{poseDiff.getOrientation(), poseDiff.getPosition()};
            rtl::RigidTf3D<double> movementCompensationTF{pose.getOrientation(), pose.getPosition()};
            auto finalTF = globalTF(startToEndTf.inverted()(movementCompensationTF(sensorOffsetTf)));

            for(size_t j = 0 ; j < noOfLasers_ ; j++) {
                const auto& point = scan->at(i*noOfLasers_ + j);
                auto& aggregator = aggregators.at(j);
                aggregator.pop_front();

                if(point.x != 0 || point.y != 0 || point.z != 0) {
                    auto tfPoint = finalTF(rtl::Vector3D<double>{point.x, point.y, point.z});
                    aggregator.push_back(tfPoint);
                } else {
                    aggregator.push_back({NAN, NAN, NAN});
                }

            }
        }

    }


    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> LaserAggregator::getAggregatedLaser(size_t laserNo) {

        auto output = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        output->reserve(aggPointsNo_);

        for(const auto& point : aggregators.at(laserNo)) {
            output->push_back({static_cast<float>(point.x()),
                               static_cast<float>(point.y()),
                               static_cast<float>(point.z())});
        }

        return output;
    }


    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> LaserAggregator::getAllAggregatedLasers() {

        auto output = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        output->reserve(aggPointsNo_ * noOfLasers_);

        for(const auto& aggregator : aggregators) {
            for (const auto &point : aggregator) {
                output->push_back({static_cast<float>(point.x()),
                                   static_cast<float>(point.y()),
                                   static_cast<float>(point.z())});
            }
        }

        return output;
    }
}