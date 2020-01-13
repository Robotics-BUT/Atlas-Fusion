#include "algorithms/pointcloud/PointCloudAggregator.h"

namespace AutoDrive::Algorithms {

    void PointCloudAggregator::addPointCloudBatches(std::vector<std::shared_ptr<DataModels::PointCloudBatch>> batches) {
        for (const auto batch : batches) {
            batchQueue_.push_back(batch);
        }
    }


    void PointCloudAggregator::filterOutBatches(uint64_t currentTime) {
        while(batchQueue_.size() > 0) {
            auto timeDiff = static_cast<double>((currentTime - batchQueue_.front()->getTimestamp()))*1e-9;
            if ( (timeDiff > aggregationTime_ )) {
                batchQueue_.pop_front();
            } else {
                break;
            }
        }
        std::cout << "batchSize: " << batchQueue_.size() << std::endl;
    }


    std::vector<std::shared_ptr<DataModels::PointCloudBatch>> PointCloudAggregator::getAllBatches() {
        std::vector<std::shared_ptr<DataModels::PointCloudBatch>> output;
        output.reserve(batchQueue_.size());

        for(auto it = batchQueue_.begin(); it < batchQueue_.end() ; it++) {
            output.push_back(*it);
        }

        return output;
    }


    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> PointCloudAggregator::getAggregatedPointCloud() {

        auto output = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

        for(int i = 0; i < batchQueue_.size() ; i++) {

            pcl::PointCloud<pcl::PointXYZ> transformedPoints;
            auto rotMat = batchQueue_.at(i)->getTF().inverted().rotQuaternion().rotMat();
            Eigen::Affine3f pcl_tf = Eigen::Affine3f::Identity();
            pcl_tf(0,0) = static_cast<float>(rotMat(0, 0)); pcl_tf(1,0) = static_cast<float>(rotMat(1, 0)); pcl_tf(2,0) = static_cast<float>(rotMat(2, 0));
            pcl_tf(0,1) = static_cast<float>(rotMat(0, 1)); pcl_tf(1,1) = static_cast<float>(rotMat(1, 1)); pcl_tf(2,1) = static_cast<float>(rotMat(2, 1));
            pcl_tf(0,2) = static_cast<float>(rotMat(0, 2)); pcl_tf(1,2) = static_cast<float>(rotMat(1, 2)); pcl_tf(2,2) = static_cast<float>(rotMat(2, 2));
            pcl_tf.translation() << (batchQueue_.at(i))->getTF().trX(), (batchQueue_.at(i))->getTF().trY(), (batchQueue_.at(i))->getTF().trZ();
            auto pts = (batchQueue_.at(i))->getPoints();
            pcl::transformPointCloud (pts, transformedPoints, pcl_tf);

            (*output) += transformedPoints;
        }
        return output;
    }

}
