#pragma once

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include "Context.h"

namespace AutoDrive::Algorithms {

    /**
     * Point Cloud Processor encapsulates simple operations on point clouds, like downsampling or applying the
     * transformation on a point cloud.
     */

    class PointCloudProcessor {

    public:

        /**
         * Constructor
         * @param context global services container (time stamps provider, TF tree, logger, etc.)
         * @param leafSize the leaf size of the downsampled point cloud
         */
        PointCloudProcessor(Context& context, float leafSize)
        : context_{context}
        , leafSize_{leafSize} {

        }

        /**
         * Method downsamples the point cloud given at the input
         * @param input given point cloud that will be downsampled
         * @return downsampled point cloud
         */
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> downsamplePointCloud(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> input);

        /**
         * Method applies 3D transpormation on a given point cloud
         * @param input input point cloud
         * @param tf 3D transformation that will be applied
         * @return transpormed point cloud
         */
        static std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> transformPointcloud(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> input, rtl::RigidTf3D<double> tf);

    private:

        Context& context_;

        float leafSize_;
    };

}