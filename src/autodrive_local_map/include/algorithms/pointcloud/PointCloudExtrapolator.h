#pragma once

#include "Context.h"

#include "data_models/lidar/LidarScanDataModel.h"
#include "data_models/local_map/PointCloudBatch.h"
#include "data_models/local_map/LocalPosition.h"

namespace AutoDrive::Algorithms {

    class PointCloudExtrapolator {


    public:

        explicit PointCloudExtrapolator(Context& context, size_t noOfBatcher)
        : context_{context}
        , noOfBatches_{noOfBatcher} {

        }

        std::vector<std::shared_ptr<DataModels::PointCloudBatch>> splitPointCloudToBatches(
                std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> scan,
                DataModels::LocalPosition startPose,
                DataModels::LocalPosition endPose,
                rtl::Transformation3D<double> sensorOffset);

        void setNoOfBatches(size_t n) {noOfBatches_ = n;};

    private:

        Context& context_;
        size_t noOfBatches_;

        DataModels::LocalPosition interpolateLocalPosition(
                DataModels::LocalPosition& begin,
                DataModels::LocalPosition& end,
                float ratio);

    };

}
