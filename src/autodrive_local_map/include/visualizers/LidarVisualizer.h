#pragma once

#include <ros/ros.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>

#include "Context.h"
#include "Topics.h"

namespace AutoDrive::Visualizers {

    class LidarVisualizer {

    public:

        enum class LidarType{
            kLeft,
            kRight,
        };

        LidarVisualizer(ros::NodeHandle& node, Context& context)
        : node_{node}
        , context_{context} {

            lidarLeftPublisher_ = node_.advertise<sensor_msgs::PointCloud2>( Topics::kLidarLeft, 0 );
            lidarRightPublisher_ = node_.advertise<sensor_msgs::PointCloud2>( Topics::kLidarRight, 0 );
            test_publisher_ = node_.advertise<sensor_msgs::PointCloud2>( "test_topic", 0 );
        }

        void drawLeftPointcloud(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>) const;
        void drawRightPointcloud(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>) const;
        void drawPointcloudOnTopic(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pc, std::string topic) const;

    private:

        ros::NodeHandle& node_;
        Context& context_;

        ros::Publisher lidarLeftPublisher_;
        ros::Publisher lidarRightPublisher_;
        ros::Publisher test_publisher_;

        void drawPointCloud(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>, const LidarType type) const;
    };
}
