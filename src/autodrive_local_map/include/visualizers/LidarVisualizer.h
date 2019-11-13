#pragma once

#include <ros/ros.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>

#include "visualizers/Topics.h"
#include "LogService.h"

namespace AutoDrive::Visualizers {

    class LidarVisualizer {

    public:

        enum class LidarType{
            kLeft,
            kRight,
        };

        LidarVisualizer(ros::NodeHandle& n, LogService& logger)
        : node_(n)
        , logger_(logger) {

            lidarLeftPublisher_ = node_.advertise<sensor_msgs::PointCloud2>( Topics::kLidarLeft, 0 );
            lidarRightPublisher_ = node_.advertise<sensor_msgs::PointCloud2>( Topics::kLidarRight, 0 );
        }

        void drawLeftPointcloud(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>) const;
        void drawRightPointcloud(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>) const;

    private:

        ros::NodeHandle& node_;
        LogService& logger_;

        ros::Publisher lidarLeftPublisher_;
        ros::Publisher lidarRightPublisher_;

        void drawPointCloud(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>, const LidarType type) const;
    };
}
