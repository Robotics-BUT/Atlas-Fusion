#include "algorithms/pointcloud/PointCloudProcessor.h"

namespace AutoDrive::Algorithms {


    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> PointCloudProcessor::downsamplePointCloud(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> input) {

        auto output = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

        pcl::VoxelGrid<pcl::PointXYZ> downsampler;
        downsampler.setInputCloud (input->makeShared());
        downsampler.setLeafSize (leafSize_, leafSize_, leafSize_);
        downsampler.filter (*output);

        return output;
    }
}