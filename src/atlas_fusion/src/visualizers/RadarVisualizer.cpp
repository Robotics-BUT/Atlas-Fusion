#include "visualizers/RadarVisualizer.h"

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

namespace AtlasFusion::Visualizers {

    void RadarVisualizer::drawRadarDetectionsOnTopic(const std::vector<DataModels::RadarTiDataModel::Object>& objects, std::string topic, std::string frame) {

        if(publishers_.count(topic) == 0) {
            publishers_[topic] = std::make_shared<ros::Publisher>(node_.advertise<visualization_msgs::MarkerArray>( topic, 0));
        }

        auto const time = ros::Time::now();
        visualization_msgs::MarkerArray msg;
        size_t object_count = 0;
        for (const auto& object : objects) {

            visualization_msgs::Marker obj;
            obj.header.frame_id = frame;
            obj.header.stamp = time;
            obj.id = object_count++;
            obj.pose.position.x = object.getPose().x();
            obj.pose.position.y = object.getPose().y();
            obj.pose.position.z = object.getPose().z();
            obj.scale.x = 0.3;
            obj.scale.y = 0.3;
            obj.scale.z = 0.3;

            obj.color.a = 1.0;
            obj.color.r = 1.0;
            obj.color.g = 0.0;
            obj.color.b = 1.0;
            obj.type = visualization_msgs::Marker::SPHERE;
            obj.action = visualization_msgs::Marker::ADD;
            msg.markers.emplace_back(obj);

            visualization_msgs::Marker textMsg;
            textMsg.header.frame_id = frame;
            textMsg.header.stamp = time;
            textMsg.id = object_count++;
            textMsg.pose.position.x = object.getPose().x();
            textMsg.pose.position.y = object.getPose().y();
            textMsg.pose.position.z = object.getPose().z() + 0.1;
            textMsg.scale.x = 0.3;
            textMsg.scale.y = 0.3;
            textMsg.scale.z = 0.3;
            textMsg.color.a = 1.0;
            textMsg.color.r = 1.0;
            textMsg.color.g = 0.0;
            textMsg.color.b = 1.0;
            textMsg.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            textMsg.action = visualization_msgs::Marker::ADD;
            std::stringstream ss;
            ss << "vel: " << object.getVelocity();
            textMsg.text = ss.str();
            msg.markers.emplace_back(textMsg);
        }

        for (size_t i = object_count ; i < radarTiMaxObjectVisCount_; i++) {
            visualization_msgs::Marker obj;
            obj.header.frame_id = frame;
            obj.header.stamp = time;
            obj.id = object_count++;
            obj.color.a = 0.0;
            obj.scale.x = 0.1;
            obj.scale.y = 0.1;
            obj.scale.z = 0.1;
            obj.type = visualization_msgs::Marker::SPHERE;
            obj.action = visualization_msgs::Marker::ADD;
            msg.markers.emplace_back(obj);
        }
        radarTiMaxObjectVisCount_ = object_count;
        
        publishers_[topic]->publish(msg);
    }
}