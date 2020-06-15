#include "algorithms/pointcloud/PointCloudProcessor.h"

#include <pcl/common/transforms.h>

namespace AutoDrive::Algorithms {


    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> PointCloudProcessor::downsamplePointCloud(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> input) {

        auto output = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

        pcl::VoxelGrid<pcl::PointXYZ> downsampler;
        downsampler.setInputCloud (input->makeShared());
        downsampler.setLeafSize (leafSize_, leafSize_, leafSize_);
        downsampler.filter (*output);

        return output;
    }

    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> PointCloudProcessor::transformPointcloud(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> input, rtl::RigidTf3D<double> tf)  {

            auto output = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
            output->reserve(input->size());

            auto rotMat = tf.rotMat();
            Eigen::Affine3f pcl_tf = Eigen::Affine3f::Identity();
            pcl_tf(0,0) = static_cast<float>(rotMat(0, 0));
            pcl_tf(1,0) = static_cast<float>(rotMat(1, 0));
            pcl_tf(2,0) = static_cast<float>(rotMat(2, 0));
            pcl_tf(0,1) = static_cast<float>(rotMat(0, 1));
            pcl_tf(1,1) = static_cast<float>(rotMat(1, 1));
            pcl_tf(2,1) = static_cast<float>(rotMat(2, 1));
            pcl_tf(0,2) = static_cast<float>(rotMat(0, 2));
            pcl_tf(1,2) = static_cast<float>(rotMat(1, 2));
            pcl_tf(2,2) = static_cast<float>(rotMat(2, 2));
            pcl_tf.translation() << tf.trVecX(), tf.trVecY(), tf.trVecZ();
            pcl::transformPointCloud (*input, *output, pcl_tf);

            return output;
    }
}