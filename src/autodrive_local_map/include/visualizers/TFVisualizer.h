#pragma once

#include "Context.h"
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <cstdio>
#include <tf2/LinearMath/Quaternion.h>

#include "local_map/Frames.h"

namespace AutoDrive::Visualizers {

    class TFVisualizer {

    public:

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
                msg.transform.translation.x = tf.trX();
                msg.transform.translation.y = tf.trY();
                msg.transform.translation.z = tf.trZ();
                msg.transform.rotation.x = tf.rotQuaternion().x();
                msg.transform.rotation.y = tf.rotQuaternion().y();
                msg.transform.rotation.z = tf.rotQuaternion().z();
                msg.transform.rotation.w = tf.rotQuaternion().w();
                msgVector.push_back(msg);
            }
            broadcaster_->sendTransform(msgVector);
        }

        void updateOriginToRootTf(rtl::Transformation3D<double> tf);

    protected:

        ros::NodeHandle& node_;
        Context& context_;
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_{};
        tf2_ros::TransformBroadcaster rootToOriginBroadcaster_{};
    };
}
