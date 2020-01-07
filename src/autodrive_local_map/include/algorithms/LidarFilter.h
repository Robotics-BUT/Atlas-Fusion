#pragma once

#include <vector>
#include <memory>
#include "data_models/lidar/LidarScanDataModel.h"

namespace AutoDrive::Algorithms {

    class LidarFilter {

    public:

        void applyFiltersOnLidarData( pcl::PointCloud<pcl::PointXYZ>& data) {

            if(filterNearObjects_) {
                filterNearObjects(data);
            }
        }

        void enableFilterNearObjects() {filterNearObjects_ = true;};
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
