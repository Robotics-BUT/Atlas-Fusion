#include "algorithms/pointcloud/PointCloudProcessor.h"

namespace AutoDrive::Algorithms {


    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> PointCloudProcessor::downsamplePointCloud(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> input) {

        auto output = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

//        std::cout << " * Downsampling PC * " << std::endl;
//        std::cout << "input points: " << input->width << std::endl;
//        auto start = context_.getHighPrecisionTime();

        pcl::VoxelGrid<pcl::PointXYZ> downsampler;
        downsampler.setInputCloud (input->makeShared());
        downsampler.setLeafSize (leafSize_, leafSize_, leafSize_);
        downsampler.filter (*output);

//        auto end = context_.getHighPrecisionTime();
//        std::cout << "ouptut points: " << output->width << std::endl;
//        std::cout << "duration: " << context_.highPrecisionTimeToMilliseconds(end-start) << std::endl;

        return output;
    }
}