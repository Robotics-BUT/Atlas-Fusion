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

#include "Context.h"
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <cstdio>
#include <tf2/LinearMath/Quaternion.h>

#include "local_map/Frames.h"

namespace AtlasFusion::Visualizers {

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
        TFVisualizer(ros::NodeHandle& node, Context& context)
        : node_{node}
        , context_{context} {

            broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>();
            std::vector<geometry_msgs::TransformStamped> msgVector;
            for (const auto& frameName : context_.tfTree_.getFrameNames()) {
                auto tf = context_.tfTree_.getTransformationForFrame(frameName);
                geometry_msgs::TransformStamped msg;
                msg.header.stamp = ros::Time::now();
                msg.header.frame_id = context_.tfTree_.getRootFrameName();
                msg.child_frame_id = frameName;
                msg.transform.translation.x = tf.trVecX();
                msg.transform.translation.y = tf.trVecY();
                msg.transform.translation.z = tf.trVecZ();
                msg.transform.rotation.x = tf.rotQuaternion().x();
                msg.transform.rotation.y = tf.rotQuaternion().y();
                msg.transform.rotation.z = tf.rotQuaternion().z();
                msg.transform.rotation.w = tf.rotQuaternion().w();
                msgVector.push_back(msg);
            }
            broadcaster_->sendTransform(msgVector);
        }

        void updateOriginToRootTf(rtl::RigidTf3D<double> tf);

    protected:

        ros::NodeHandle& node_;
        Context& context_;
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_{};
        tf2_ros::TransformBroadcaster rootToOriginBroadcaster_{};
    };
}
