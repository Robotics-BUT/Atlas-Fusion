#include "data_loader/data_models/lidar/LidarScanDataModel.h"

#include <sstream>

namespace AutoDrive {
    namespace DataLoader {

        std::string LidarScanDataModel::toString() {

            std::stringstream ss;
            ss << "[Lidar Scan Data Model] : "
               << scan_path_ << scan_.size();
            return ss.str();
        }

        pcl::PointCloud<pcl::PointXYZ> LidarScanDataModel::getScan() {

            if (pcl::io::loadPCDFile<pcl::PointXYZ> (scan_path_, scan_) == -1) {
                std::cerr << "Could not open pcd file: " << scan_path_ << std::endl;
            }
            return scan_;
        }

    }
}
