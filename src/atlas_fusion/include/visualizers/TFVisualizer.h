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

#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "util/IdentifierToFrameConversions.h"
#include <cstdio>


namespace AutoDrive::Visualizers {

    /**
     * Visualization backend (ROS) implementations for visualizing transformation tree
     */
    class TFVisualizer {

    public:

        /**
         * Constructor
         * @param node ros node reference
         * @param context global services container (timestamps, logging, etc.)
         */
        TFVisualizer(rclcpp::Node::SharedPtr &node, Context &context) :
        node_{node}
        , context_{context}
        , broadcaster_(node_)
        , rootToOriginBroadcaster_(node_) {

            std::vector<geometry_msgs::msg::TransformStamped> msgVector;
            for (const auto &frameType: context_.tfTree_.getFrameTypes()) {
                auto tf = context_.tfTree_.getTransformationForFrame(frameType);
                geometry_msgs::msg::TransformStamped msg;
                msg.header.stamp = node_->get_clock()->now();
                msg.header.frame_id = frameTypeName(context_.tfTree_.getRootFrameType());
                msg.child_frame_id = frameTypeName(frameType);
                msg.transform.translation.x = tf.trVecX();
                msg.transform.translation.y = tf.trVecY();
                msg.transform.translation.z = tf.trVecZ();
                msg.transform.rotation.x = tf.rotQuaternion().x();
                msg.transform.rotation.y = tf.rotQuaternion().y();
                msg.transform.rotation.z = tf.rotQuaternion().z();
                msg.transform.rotation.w = tf.rotQuaternion().w();
                msgVector.push_back(msg);
            }
            broadcaster_.sendTransform(msgVector);
        }

        void updateOriginToRootTf(const rtl::RigidTf3D<double> &tf);

    protected:

        rclcpp::Node::SharedPtr &node_;
        Context &context_;
        tf2_ros::StaticTransformBroadcaster broadcaster_;
        tf2_ros::TransformBroadcaster rootToOriginBroadcaster_;
    };
}
