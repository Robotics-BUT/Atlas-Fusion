#pragma once

#include "Context.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "algorithms/pointcloud/PointCloudProcessor.h"

namespace AutoDrive::Algorithms {

    /**
     *  Class is designed as a container for asynchronous point cloud aggregation during the entire mapping session
     *  In this way the global point cloud map is created.
     */

    class GlobalPointcloudStorage {

    public:

        GlobalPointcloudStorage() = delete;

       /**
        * Constructor
        * @param context contains global services, like logger, TF tree or calibration loader
        * @param leafSize leaf size of the output point cloud
        */
        GlobalPointcloudStorage(Context& context, float leafSize)
                : context_{context}
                , processor_{context, leafSize} {

            globalStorage_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        }

        /**
         * Method accepts new point clouds and aggregates them into the global model
         * @param pc new points
         */
        void addMorePointsToGlobalStorage(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pc);

        /**
         * Getter for all the currently aggregated points.
         * @return donwsampled global point cloud map with a leaf size defined from constructor
         */
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> getEntirePointcloud();

    private:

        Context& context_;

        PointCloudProcessor processor_;
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> globalStorage_;
    };

}

