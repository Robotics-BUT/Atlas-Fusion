/*
 * Copyright 2020 Brno University of Technology
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom
 * the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT
 * OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
 * OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include "visualizers/TelemetryVisualizer.h"

#include "visualization_msgs/Marker.h"

namespace AtlasFusion::Visualizers {


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
