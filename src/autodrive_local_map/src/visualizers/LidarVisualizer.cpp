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


    void LidarVisualizer::drawLidarDetections(std::vector<std::shared_ptr<const DataModels::LidarDetection>> detections, std::string topic, std::string frame) {

        if(publishers_.count(topic) == 0) {
            publishers_[topic] = std::make_shared<ros::Publisher>(node_.advertise<visualization_msgs::MarkerArray>( topic, 0));
        }
        publishers_[topic]->publish(lidarDetectionsToMarkerArray(detections, frame));
    }


    visualization_msgs::MarkerArray LidarVisualizer::lidarDetectionsToMarkerArray(std::vector<std::shared_ptr<const DataModels::LidarDetection>> detections, std::string frame) {

        visualization_msgs::MarkerArray output;

        auto timestamp = ros::Time::now();

        size_t cnt = 0;
        static size_t maxCnt = 0;

        for(const auto& detection : detections) {

            double dx = detection->getBoundingBox().max().getElement(0) - detection->getBoundingBox().min().getElement(0);
            double dy = detection->getBoundingBox().max().getElement(1) - detection->getBoundingBox().min().getElement(1);
            double dz = detection->getBoundingBox().max().getElement(2) - detection->getBoundingBox().min().getElement(2);

            double cx = (detection->getBoundingBox().max().getElement(0) + detection->getBoundingBox().min().getElement(0)) / 2;
            double cy = (detection->getBoundingBox().max().getElement(1) + detection->getBoundingBox().min().getElement(1)) / 2;
            double cz = (detection->getBoundingBox().max().getElement(2) + detection->getBoundingBox().min().getElement(2)) / 2;

            // Bounding Box
            visualization_msgs::Marker marker;
            marker.header.frame_id = frame;
            marker.header.stamp = timestamp;
            marker.id = cnt++;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;

            marker.pose.position.x = cx;
            marker.pose.position.y = cy;
            marker.pose.position.z = cz;

            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;

            marker.scale.x = dx;
            marker.scale.y = dy;
            marker.scale.z = dz;

            marker.color.a = 0.6;
            marker.color.r = 0.7;
            marker.color.g = 0.7;
            marker.color.b = 0.7;

            output.markers.push_back(marker);

            visualization_msgs::Marker text;
            text.header.frame_id = frame;
            text.header.stamp = timestamp;
            text.id = cnt++;
            text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            text.action = visualization_msgs::Marker::ADD;

            text.pose.position.x = detection->getBoundingBox().max().getElement(0);
            text.pose.position.y = detection->getBoundingBox().max().getElement(1);
            text.pose.position.z = detection->getBoundingBox().max().getElement(2);

            std::stringstream sstream;
            sstream << "ID: " << detection->getID() << std::endl
                    << "TTL: " << detection->getTTL() << std::endl;
            text.text = sstream.str();

            text.scale.x = 1;
            text.scale.y = 1;
            text.scale.z = 1;

            text.color.a = 0.6;
            text.color.r = 0.7;
            text.color.g = 0.7;
            text.color.b = 0.7;

            output.markers.push_back(text);
        }

        for(size_t i = cnt ; i < maxCnt ; i++) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = frame;
            marker.header.stamp = timestamp;
            marker.id = cnt++;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.color.a = 0.0;

            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;

            output.markers.push_back(marker);
        }
        maxCnt = cnt;
        return output;
    }
}