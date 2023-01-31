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
#include "std_msgs/msg/color_rgba.hpp"

namespace AutoDrive::Visualizers {

    void SensorStatusVisualizer::publishStatusAsText(const std::string &status, const std::string &topic) {
        if (textPublishers_.count(topic) == 0) {
            textPublishers_[topic] = node_->create_publisher<rviz_2d_overlay_msgs::msg::OverlayText>(topic, 0);
        }

        std_msgs::msg::ColorRGBA color;
        color.r = 1.0;
        color.g = 0.0;
        color.b = 0.0;
        color.a = 1.0;

        rviz_2d_overlay_msgs::msg::OverlayText textMsg;
        textMsg.width = 400;
        textMsg.height = 600;
        textMsg.left = 10;
        textMsg.top = 10;
        textMsg.text_size = 8;
        textMsg.line_width = 2;
        textMsg.font = "DejaVu Sans Mono";
        textMsg.fg_color = color;
        textMsg.text = status;

        textPublishers_[topic]->publish(textMsg);
    }

    void SensorStatusVisualizer::publishStatusAsText(const FailCheck::SensorStatus &status, const std::string &topic) {
        publishStatusAsText(status.statusString, topic);
    }

    void SensorStatusVisualizer::publishStatusAsList(const FailCheck::SensorStatus &status, const std::string &topic) {
        if (listPublishers_.count(topic) == 0) {
            listPublishers_[topic] = node_->create_publisher<std_msgs::msg::Float32MultiArray>(topic, 0);
        }

        std_msgs::msg::Float32MultiArray array_msg;

        for(const auto& s : status.statusVector) {
            array_msg.data.push_back(s);
        }

        listPublishers_[topic]->publish(array_msg);
    }
}
