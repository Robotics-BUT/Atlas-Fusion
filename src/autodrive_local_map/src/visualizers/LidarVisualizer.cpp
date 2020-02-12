#include "visualizers/LidarVisualizer.h"

#include <pcl_conversions/pcl_conversions.h>
#include "local_map/Frames.h"

namespace AutoDrive::Visualizers {

    void LidarVisualizer::drawPointcloudOnTopic(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pc, std::string topic, std::string frame) {

        if(publishers_.count(topic) == 0) {
            publishers_[topic] = std::make_shared<ros::Publisher>(node_.advertise<sensor_msgs::PointCloud2>( topic, 0));
        }

        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(*pc, msg);

        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = frame;
        publishers_[topic]->publish(msg);
    }
}