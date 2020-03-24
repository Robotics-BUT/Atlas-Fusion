#pragma once

#include <queue>
#include <rtl/BoundingBox.h>

#include "Context.h"
#include "data_models/local_map/PointCloudBatch.h"
#include "data_models/local_map/LocalPosition.h"

namespace AutoDrive::Algorithms {

    class PointCloudAggregator {

    public:

        explicit PointCloudAggregator(Context& context, float aggTime)
        : context_{context}
        , aggregationTime_{aggTime} {

        }

        void addPointCloudBatches(std::vector<std::shared_ptr<DataModels::PointCloudBatch>> batches);
        void filterOutBatches(uint64_t currentTime);
        std::vector<std::shared_ptr<DataModels::PointCloudBatch>> getAllBatches();
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> getAggregatedPointCloud();

        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> getPointcloudCutout(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> input, rtl::BoundingBox3f);

    private:

        Context& context_;

        double aggregationTime_;
        std::deque<std::shared_ptr<DataModels::PointCloudBatch>> batchQueue_;

    };
}
