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

#include "visualizers/SensorStatusVisualizer.h"

#include "visualization_msgs/Marker.h"
#include "util/IdentifierToFrameConversions.h"

namespace AutoDrive::Visualizers {


    void SensorStatusVisualizer::drawStatusAsText(const std::string &statusText, const FrameType &frame, const std::string &topic) {
        if (publishers_.count(topic) == 0) {
            publishers_[topic] = node_.advertise<visualization_msgs::Marker>(topic, 0);
        }

        auto pos = getTextPosition(frame);

        visualization_msgs::Marker textMsg;
        textMsg.header.frame_id = frameTypeName(frame);
        textMsg.header.stamp = ros::Time::now();
        textMsg.pose.position.x = std::get<0>(pos);
        textMsg.pose.position.y = std::get<1>(pos);
        textMsg.pose.position.z = std::get<2>(pos);
        textMsg.scale.x = 0.2;
        textMsg.scale.y = textMsg.scale.x;
        textMsg.scale.z = textMsg.scale.x;
        textMsg.color.a = 1.0;
        textMsg.color.r = 1.0;
        textMsg.color.g = 0.0;
        textMsg.color.b = 0.0;
        textMsg.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        textMsg.action = visualization_msgs::Marker::ADD;

        textMsg.text = statusText;

        publishers_[topic].publish(textMsg);
    }

    std::tuple<double, double, double> SensorStatusVisualizer::getTextPosition(const FrameType &frame) const {
        std::tuple<double, double, double> pos(0.0, 0.0, 0.0);
        switch (frame) {
            case FrameType::kOrigin:
            case FrameType::kLidarLeft:
            case FrameType::kLidarRight:
            case FrameType::kLidarCenter:
            case FrameType::kRadarTi:
            case FrameType::kGnssAntennaFront:
            case FrameType::kGnssAntennaRear:
            case FrameType::kImu:
                return pos;
            case FrameType::kCameraLeftFront:
                std::get<0>(pos) = -0.5; std::get<1>(pos) = -2.0; std::get<2>(pos) = 1.5;
                break;
            case FrameType::kCameraLeftSide:
                std::get<0>(pos) = -1.0; std::get<1>(pos) = -2.0; std::get<2>(pos) = 1.5;
                break;
            case FrameType::kCameraRightFront:
                std::get<0>(pos) = 1.0; std::get<1>(pos) = -2.0; std::get<2>(pos) = 1.5;
                break;
            case FrameType::kCameraRightSide:
                std::get<0>(pos) = 1.0; std::get<1>(pos) = -2.0; std::get<2>(pos) = 1.5;
                break;
            case FrameType::kCameraIr:
                std::get<0>(pos) = -2.0; std::get<1>(pos) = -2.0; std::get<2>(pos) = 0.0;
                break;
        }

        return pos;
    }
}
