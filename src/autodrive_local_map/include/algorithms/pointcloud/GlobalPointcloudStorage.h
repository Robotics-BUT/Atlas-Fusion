#pragma once

#include "Context.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "algorithms/pointcloud/PointCloudProcessor.h"

namespace AutoDrive::Algorithms {

    class GlobalPointcloudStorage {

    public:

        GlobalPointcloudStorage() = delete;

        GlobalPointcloudStorage(Context& context, float leafSize)
                : context_{context}
                , processor_{context, leafSize} {

            globalStorage_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        }

        void addMorePointsToGlobalStorage(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pc);

        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> getEntirePointcloud();

    private:

        Context& context_;

        PointCloudProcessor processor_;
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> globalStorage_;
    };

}

