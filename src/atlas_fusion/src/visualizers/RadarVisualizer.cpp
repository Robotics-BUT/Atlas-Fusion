#include "visualizers/RadarVisualizer.h"
#include "util/IdentifierToFrameConversions.h"

namespace AutoDrive::Visualizers {

    void RadarVisualizer::drawRadarDetectionsOnTopic(const std::vector<DataModels::RadarTiDataModel::Object> &objects, const std::string &topic,
                                                     const FrameType &frame) {

        if (publishers_.count(topic) == 0) {
            publishers_[topic] = node_->create_publisher<visualization_msgs::msg::MarkerArray>(topic, 0);
        }

        auto const time = node_->get_clock()->now();
        visualization_msgs::msg::MarkerArray msg;
        size_t object_count = 0;
        for (const auto &object: objects) {

            visualization_msgs::msg::Marker obj;
            obj.header.frame_id = frameTypeName(frame);
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
            obj.type = visualization_msgs::msg::Marker::SPHERE;
            obj.action = visualization_msgs::msg::Marker::ADD;
            msg.markers.emplace_back(obj);

            visualization_msgs::msg::Marker textMsg;
            textMsg.header.frame_id = frameTypeName(frame);
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
            textMsg.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            textMsg.action = visualization_msgs::msg::Marker::ADD;
            std::stringstream ss;
            ss << "vel: " << object.getVelocity();
            textMsg.text = ss.str();
            msg.markers.emplace_back(textMsg);
        }

        for (size_t i = object_count; i < radarTiMaxObjectVisCount_; i++) {
            visualization_msgs::msg::Marker obj;
            obj.header.frame_id = frameTypeName(frame);
            obj.header.stamp = time;
            obj.id = object_count++;
            obj.color.a = 0.0;
            obj.scale.x = 0.1;
            obj.scale.y = 0.1;
            obj.scale.z = 0.1;
            obj.type = visualization_msgs::msg::Marker::SPHERE;
            obj.action = visualization_msgs::msg::Marker::ADD;
            msg.markers.emplace_back(obj);
        }
        radarTiMaxObjectVisCount_ = object_count;
        // std::cout << object_count << " " << radarTiMaxObjectVisCount_ << std::endl;

        publishers_[topic]->publish(msg);
    }
}