#include "algorithms/pointcloud/LaserAggregator.h"

namespace AutoDrive::Algorithms {

    void LaserAggregator::onNewLaserData(
            std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> scan,
            DataModels::LocalPosition startPose,
            DataModels::LocalPosition poseDiff,
            rtl::Transformation3D<double> sensorOffsetTf) {

        auto timeOffset = startPose.getTimestamp();
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

            auto pose = AutoDrive::DataModels::LocalPosition {
                    {poseDiff.getPosition().x() * (ratio), poseDiff.getPosition().y() * (ratio), poseDiff.getPosition().z() * (ratio)},
                    {poseDiff.getOrientation().slerp({}, (float)(1-ratio))},
                    uint64_t(duration * (ratio))
            };

            rtl::Transformation3D<double> globalTF{endPose.getOrientation(), endPose.getPosition()};
            rtl::Transformation3D<double> startToEndTf{poseDiff.getOrientation(), poseDiff.getPosition()};
            rtl::Transformation3D<double> movementCompensationTF{pose.getOrientation(), pose.getPosition()};
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
            output->push_back({point.x(), point.y(), point.z()});
        }

        return output;
    }


    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> LaserAggregator::getAllAggregatedLasers() {

        auto output = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        output->reserve(aggPointsNo_ * noOfLasers_);

        for(const auto& aggregator : aggregators) {
            for (const auto &point : aggregator) {
                output->push_back({point.x(), point.y(), point.z()});
            }
        }

        return output;
    }

}