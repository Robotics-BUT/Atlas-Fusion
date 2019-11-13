#pragma once

#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <cstdio>
#include <tf2/LinearMath/Quaternion.h>

#include "data_loader/data_models/TFFrame.h"
#include "LogService.h"
#include "TFTree.h"

namespace AutoDrive::Visualizers {

    class TFVisualizer {

    public:

        TFVisualizer(TFTree& tfTree, LogService& logger)
        : tfTree_(tfTree)
        , logger_(logger) {

            for (const auto& frameName : tfTree.getFrameNames()) {

                auto broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>();
                geometry_msgs::TransformStamped static_transformStamped;

                auto& tf = tfTree.getTree().at(frameName);

                static_transformStamped.header.stamp = ros::Time::now();
                static_transformStamped.header.frame_id = tfTree.getRootFrameName();
                static_transformStamped.child_frame_id = frameName;
                static_transformStamped.transform.translation.x = tf.trX();
                static_transformStamped.transform.translation.y = tf.trY();
                static_transformStamped.transform.translation.z = tf.trZ();
                static_transformStamped.transform.rotation.x = tf.rotQuaternion().x();
                static_transformStamped.transform.rotation.y = tf.rotQuaternion().y();
                static_transformStamped.transform.rotation.z = tf.rotQuaternion().z();
                static_transformStamped.transform.rotation.w = tf.rotQuaternion().w();
                broadcaster->sendTransform(static_transformStamped);
                broadcasters_.emplace_back(broadcaster);
            }
        }

    protected:

        std::vector<std::string> keys_{};
        std::unordered_map<std::string, AutoDrive::DataLoader::TFFrame> frames_;

        TFTree& tfTree_;
        LogService& logger_;
        std::vector<std::shared_ptr<tf2_ros::StaticTransformBroadcaster>> broadcasters_{};
    };
}
