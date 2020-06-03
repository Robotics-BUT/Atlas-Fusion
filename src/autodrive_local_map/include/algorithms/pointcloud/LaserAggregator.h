#pragma once

#include "Context.h"
#include "data_models/lidar/LidarScanDataModel.h"
#include "data_models/local_map/LocalPosition.h"

namespace AutoDrive::Algorithms {

    /**
     * Class designed to accept the point cloud scan and to split it into N separated lases and to remember M points
     * from laser's measurement history. Class also performs lidar-motion undistortion.
     */

    class LaserAggregator {

    public:

        LaserAggregator() = delete;

        /**
         * Constructor
         * @param context global services container, like loger, etc.
         * @param noOfLasers Number of lasers in the given scan
         * @param aggPointsNo number of aggregated points in the laser's history
         */
        LaserAggregator(Context& context, size_t noOfLasers, size_t aggPointsNo)
        : context_{context}
        , noOfLasers_{noOfLasers}
        , aggPointsNo_{aggPointsNo} {

            aggregators.resize(noOfLasers_);
            for(auto& agg : aggregators) {
                agg.resize(aggPointsNo_);
            }
        }

        /**
         * Method that updates class instance with a new measurements
         * @param scan new scan from lidar
         * @param startPose Position of the car center in the moment when the scan started
         * @param endPose Position of the car center in the moment when the scan ended
         * @param sensorOffset relative position of the lidar w.r.t. the car center
         */
        void onNewLaserData(
                std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> scan,
                DataModels::LocalPosition startPose,
                DataModels::LocalPosition endPose,
                rtl::Transformation3D<double> sensorOffset);

        /**
         * Method provides aggregated history for a laser with the given index
         * @param laserNo index of the requested laser history
         * @return point cloud as a history of lasers measurements
         */
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> getAggregatedLaser(size_t laserNo);

        /**
         * Method provides aggregated history for all lasers
         * @return point cloud as a history of lasers measurements
         */
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> getAllAggregatedLasers();


    private:

        Context& context_;
        size_t aggPointsNo_;
        size_t noOfLasers_;

        std::vector<std::deque<rtl::Vector3D<double>>> aggregators;

    };

}
