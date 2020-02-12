#include "algorithms/pointcloud/GlobalPointcloudStorage.h"

namespace AutoDrive::Algorithms {

    void GlobalPointcloudStorage::addMorePointsToGlobalStorage(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pc) {


        *globalStorage_ += *pc;
    }

    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> GlobalPointcloudStorage::getEntirePointcloud() {

        globalStorage_ = processor_.downsamplePointCloud(globalStorage_);
        return globalStorage_;
    }
}
