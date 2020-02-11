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
        if(filteredScan_.points.empty()) {
            if(scan_.points.empty()) {
                getRawScan();
            }
            filteredScan_ = scan_;
            filter_(filteredScan_);
        }

        return std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(filteredScan_);
    }


    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> LidarScanDataModel::getRawScan() {
        if(scan_.points.empty()) {
            if (pcl::io::loadPCDFile<pcl::PointXYZ>(scan_path_, scan_) == -1) {
                std::cerr << "Could not open pcd file: " << scan_path_ << std::endl;
            }
        }
        return std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(scan_);
    }

    void LidarScanDataModel::addPointToScan(pcl::PointXYZ point) {
        scan_.push_back(point);
    };
}
