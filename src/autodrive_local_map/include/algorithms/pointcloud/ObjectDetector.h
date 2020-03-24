#pragma once

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include "Context.h"
#include "data_models/local_map/LidarDetection.h"

namespace AutoDrive::Algorithms {

    class ObjectDetector {

    public:

        ObjectDetector() = delete;
        ObjectDetector(Context& context)
        : context_{context} {}

        std::vector<std::shared_ptr<const DataModels::LidarDetection>> detectObstacles(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>);

    private:

        Context& context_;

    };

}
