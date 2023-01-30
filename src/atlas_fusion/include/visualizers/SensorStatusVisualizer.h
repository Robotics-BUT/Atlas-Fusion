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

#pragma once

#include "rviz_2d_overlay_msgs/msg/overlay_text.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "fail_check/AbstractFailChecker.h"

namespace AutoDrive::Visualizers {

    /**
     * Visualization backend (ROS) implementations for visualizing text status
     */
    class SensorStatusVisualizer {

    public:

        SensorStatusVisualizer() = delete;

        /**
         * Constructor
         * @param node ros node reference
         * @param context global services container (timestamps, logging, etc.)
         * @param frameType sensor identification
         */
        SensorStatusVisualizer(rclcpp::Node::SharedPtr &node, Context &context) : node_{node}, context_{context} {}

        void publishStatusAsText(const FailCheck::SensorStatus &status, const std::string &topic);

        void publishStatusAsList(const FailCheck::SensorStatus &status, const std::string &topic);

    private:

        rclcpp::Node::SharedPtr &node_;
        Context &context_;

        std::map<std::string, rclcpp::Publisher<rviz_2d_overlay_msgs::msg::OverlayText>::SharedPtr> textPublishers_;
        std::map<std::string, rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr> listPublishers_;

    };

}
