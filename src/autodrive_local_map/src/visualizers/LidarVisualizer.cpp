#include "visualizers/LidarVisualizer.h"

#include <pcl_conversions/pcl_conversions.h>
#include "visualizers/Frames.h"

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
                msg.header.frame_id = Frames::kLidarLeft;
                lidarLeftPublisher_.publish(msg);
                break;
            case LidarType::kRight:
                msg.header.frame_id = Frames::kLidarRight;
                lidarRightPublisher_.publish(msg);
                break;
        }
    }
}