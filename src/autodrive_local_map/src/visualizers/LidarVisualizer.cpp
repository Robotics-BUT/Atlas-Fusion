#include "visualizers/LidarVisualizer.h"

#include <pcl_conversions/pcl_conversions.h>
#include "local_map/Frames.h"

namespace AutoDrive::Visualizers {


    void LidarVisualizer::drawLeftPointcloud(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pc) const {
        drawPointCloud(pc, LidarType::kLeft);
    }


    void LidarVisualizer::drawRightPointcloud(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pc) const {
        drawPointCloud(pc, LidarType::kRight);
    }

    void LidarVisualizer::drawPointCloud(
            const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pc,
            const LidarType type) const {

        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(*pc, msg);

        msg.header.stamp = ros::Time::now();

        switch (type) {
            case LidarType::kLeft:
                msg.header.frame_id = LocalMap::Frames::kLidarLeft;
                lidarLeftPublisher_.publish(msg);
                break;
            case LidarType::kRight:
                msg.header.frame_id = LocalMap::Frames::kLidarRight;
                lidarRightPublisher_.publish(msg);
                break;
        }
    }

    void LidarVisualizer::drawPointcloudOnTopic(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pc, const std::string topic) const {
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(*pc, msg);

        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = LocalMap::Frames::kOrigin;
        test_publisher_.publish(msg);
    }


    void LidarVisualizer::drawLasers(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pc) const {
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(*pc, msg);

        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = LocalMap::Frames::kOrigin;
        laser_publisher_.publish(msg);
    }
}