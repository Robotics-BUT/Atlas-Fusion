#include "data_models/lidar/LidarScanDataModel.h"

#include <sstream>

namespace AutoDrive::DataModels {

    std::string LidarScanDataModel::toString() {

        std::stringstream ss;
        ss << "[Lidar Scan Data Model] : "
           << scan_path_ << scan_.size();
        return ss.str();
    }

    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> LidarScanDataModel::getScan() {
        if(scan_.points.empty()) {
//            scan_.push_back({1,-10,0.1});
            if (pcl::io::loadPCDFile<pcl::PointXYZ>(scan_path_, scan_) == -1) {
                std::cerr << "Could not open pcd file: " << scan_path_ << std::endl;
            }
        }
        if(filter_ != nullptr) {
            filter_(scan_);
        }

        return std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(scan_);
    }

    void LidarScanDataModel::addPointToScan(pcl::PointXYZ point) {
        scan_.push_back(point);
    };
}
