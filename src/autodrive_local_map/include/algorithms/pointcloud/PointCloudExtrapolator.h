#pragma once

#include "Context.h"

#include "data_models/lidar/LidarScanDataModel.h"
#include "data_models/local_map/PointCloudBatch.h"
#include "data_models/local_map/LocalPosition.h"

namespace AutoDrive::Algorithms {

    /**
     * Point Cloud Extrapolator takes point cloud scan on the input and based on the given poisition in which the the
     * scanner was at the very beginning of the scanning and at very end in splits the scan into N batches and linearly
     * interpolates points's poisition. In this way the lidar-motion undistortion is done.
     */
    class PointCloudExtrapolator {

    public:

        /**
         * Contructor
         * @param context global services container, like time, TF tree, etc.
         * @param noOfBatcher number of batches the scan will be splitted into
         */
        explicit PointCloudExtrapolator(Context& context, size_t noOfBatcher)
        : context_{context}
        , noOfBatches_{noOfBatcher} {

        }

        /**
         * Process single scan and performs lidar-motion undistortion
         * @param scan input lidar scan
         * @param startPose car position at the moment the scan started
         * @param poseDiff car position at the end of scan
         * @param sensorOffset sensor to car offset
         * @return Returns the vector of shared pointers on batches that represents the undistorted input scan
         */
        std::vector<std::shared_ptr<DataModels::PointCloudBatch>> splitPointCloudToBatches(
                std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> scan,
                DataModels::LocalPosition startPose,
                DataModels::LocalPosition poseDiff,
                rtl::Transformation3D<double> sensorOffset);

        /**
         * Setter for internal variable that defines number of batches that the input scan will be splitted into.
         * @param n nubmer of batches
         */
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
