#pragma once

#include "Context.h"
#include "data_models/lidar/LidarScanDataModel.h"
#include "data_models/local_map/LocalPosition.h"

namespace AutoDrive::Algorithms {

    class LaserAggregator {

    public:

        LaserAggregator() = delete;

        LaserAggregator(Context& context, size_t noOfLasers, size_t aggPointsNo)
        : context_{context}
        , noOfLasers_{noOfLasers}
        , aggPointsNo_{aggPointsNo} {

            aggregators.resize(noOfLasers_);
            for(auto& agg : aggregators) {
                agg.resize(aggPointsNo_);
            }
        }

        void onNewLaserData(
                std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> scan,
                DataModels::LocalPosition startPose,
                DataModels::LocalPosition endPose,
                rtl::Transformation3D<double> sensorOffset);

        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> getAggregatedLaser(size_t laserNo);
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> getAllAggregatedLasers();


    private:

        Context& context_;
        size_t aggPointsNo_;
        size_t noOfLasers_;

        std::vector<std::deque<rtl::Vector3D<double>>> aggregators;

    };

}
