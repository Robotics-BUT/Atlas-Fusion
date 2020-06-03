#pragma once

#include <vector>
#include <memory>
#include "data_models/lidar/LidarScanDataModel.h"

namespace AutoDrive::Algorithms {

    /**
     * Lidar Filter class encapsulates simple tools that allows to filter input point cloud data that comes directly
     * from the lidar scanner
     */
    class LidarFilter {

    public:

        /**
         * Method calls the set of filtering actions on the incomming lidar scan
         * @param data lidar scan
         */
        void applyFiltersOnLidarData( pcl::PointCloud<pcl::PointXYZ>& data) {

            if(filterNearObjects_) {
                filterNearObjects(data);
            }
        }

        /**
         * Set the flag that enables point cloud filtration
         */
        void enableFilterNearObjects() {filterNearObjects_ = true;};

        /**
         * Reset the flag that enables point cloud filtration
         */
        void disableFilterNearObjects() {filterNearObjects_ = false;};

    private:

        bool filterNearObjects_ = false;
        void filterNearObjects(pcl::PointCloud<pcl::PointXYZ>& data) {
            auto backup = data.points;
            data.points.clear();

            for(const auto& point : backup) {
                if(std::abs(point.x) + std::abs(point.y) > 2.0) {
                    data.points.push_back(point);
                }
            }
            data.width = data.size();
        }

    };

}
