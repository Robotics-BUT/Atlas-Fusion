#include "algorithms/pointcloud/PointCloudAggregator.h"

namespace AutoDrive::Algorithms {

    void PointCloudAggregator::addPointCloudBatches(std::vector<std::shared_ptr<DataModels::PointCloudBatch>> batches) {
        for (const auto& batch : batches) {
            batchQueue_.push_back(batch);
        }
    }


    void PointCloudAggregator::filterOutBatches(uint64_t currentTime) {


//        std::cout << " * Filtering batches * " << std::endl;
//        std::cout << "batches no: " << batchQueue_.size() << std::endl;
//        auto start = Context::getHighPrecisionTime();

        while(!batchQueue_.empty()) {
            auto timeDiff = static_cast<double>((currentTime - batchQueue_.front()->getTimestamp()))*1e-9;
            if ( (timeDiff > aggregationTime_ )) {
                batchQueue_.pop_front();
            } else {
                break;
            }
        }

//        auto end = Context::getHighPrecisionTime();
//        std::cout << "duration: " << Context::highPrecisionTimeToMilliseconds(end-start) << std::endl;
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

//        std::cout << " * Get aggregated points * " << std::endl;
//        std::cout << "batches no: " << batchQueue_.size() << std::endl;
//        auto start = Context::getHighPrecisionTime();

        auto output = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

        if(!batchQueue_.empty()) {
            output->reserve(batchQueue_.size() * 2 * batchQueue_.at(0)->getPointsSize());

            for (const auto &batch : batchQueue_) {
                // TODO: Avoid using + operator
                *output += *(batch->getTransformedPoints());
            }

//            auto end = Context::getHighPrecisionTime();
//            std::cout << "duration: " << Context::highPrecisionTimeToMilliseconds(end - start) << std::endl;
        }

        return output;
    }

}
