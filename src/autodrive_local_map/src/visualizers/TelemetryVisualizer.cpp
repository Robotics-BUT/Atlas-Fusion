#include "visualizers/TelemetryVisualizer.h"

#include "visualization_msgs/Marker.h"

namespace AutoDrive::Visualizers {


    void TelemetryVisualizer::drawTelemetryAsText(std::string& telemetryText, std::string frame, std::string topic) {

        if(publishers_.count(topic) == 0){
            publishers_[topic] = node_.advertise<visualization_msgs::Marker>( topic, 0 );
        }

        visualization_msgs::Marker textMsg;
        textMsg.header.frame_id = frame;
        textMsg.header.stamp = ros::Time::now();
        textMsg.pose.position.x = 0.0;
        textMsg.pose.position.y = -3;
        textMsg.pose.position.z = -1.0;
        textMsg.scale.x = 0.2;
        textMsg.scale.y = textMsg.scale.x;
        textMsg.scale.z = textMsg.scale.x;
        textMsg.color.a = 1.0;
        textMsg.color.r = 1.0;
        textMsg.color.g = 0.0;
        textMsg.color.b = 0.0;
        textMsg.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        textMsg.action = visualization_msgs::Marker::ADD;

        textMsg.text = telemetryText;

        publishers_[topic].publish(textMsg);
    }
}
