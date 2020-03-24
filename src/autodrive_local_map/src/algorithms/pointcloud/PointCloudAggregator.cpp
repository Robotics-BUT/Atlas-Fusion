#include "algorithms/pointcloud/PointCloudAggregator.h"

namespace AutoDrive::Algorithms {

    void PointCloudAggregator::addPointCloudBatches(std::vector<std::shared_ptr<DataModels::PointCloudBatch>> batches) {
        for (const auto& batch : batches) {
            batchQueue_.push_back(batch);
        }
    }


    void PointCloudAggregator::filterOutBatches(uint64_t currentTime) {

        while(!batchQueue_.empty()) {
            auto timeDiff = static_cast<double>((currentTime - batchQueue_.front()->getTimestamp()))*1e-9;
            if ( (timeDiff > aggregationTime_ )) {
                batchQueue_.pop_front();
            } else {
                break;
            }
        }
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

        if(!batchQueue_.empty()) {
            output->reserve(batchQueue_.size() * 2 * batchQueue_.at(0)->getPointsSize());

            for (const auto &batch : batchQueue_) {
                // TODO: Avoid using + operator
                *output += *(batch->getTransformedPoints());
            }
        }
        return output;
    }



    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> PointCloudAggregator::getPointcloudCutout(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> input,rtl::BoundingBox3f borders) {

        auto output = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        output->reserve(input->size());

        for(const auto& point : *input) {
            if(point.x > borders.min().getElement(0) && point.x < borders.max().getElement(0) &&
               point.y > borders.min().getElement(1) && point.y < borders.max().getElement(1) &&
               point.z > borders.min().getElement(2) && point.z < borders.max().getElement(2)) {
                output->push_back(point);
            }
        }
        return output;
    }

}
