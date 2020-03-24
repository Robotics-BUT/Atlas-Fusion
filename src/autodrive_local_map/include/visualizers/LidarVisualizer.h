#pragma once

#include <ros/ros.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

#include "Context.h"
#include "Topics.h"
#include "data_models/local_map/LidarDetection.h"

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

        }


        void drawPointcloudOnTopic(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pc, std::string topic, std::string frame);
        void drawLidarDetections(std::vector<std::shared_ptr<const DataModels::LidarDetection>> detections, std::string topic, std::string frame);

    private:

        ros::NodeHandle& node_;
        Context& context_;

        std::map<std::string, std::shared_ptr<ros::Publisher>> publishers_;

        visualization_msgs::MarkerArray lidarDetectionsToMarkerArray(std::vector<std::shared_ptr<const DataModels::LidarDetection>> detections, std::string frame);
    };
}
