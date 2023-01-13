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

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "Context.h"
#include "Topics.h"
#include "data_models/local_map/LidarDetection.h"

namespace AutoDrive::Visualizers {

    /**
     * Visualization backend (ROS) implementations for visualizing point cloud structures, like raw lidar scans or
     * aggregated point clouds
     */
    class LidarVisualizer {

    public:

        /**
         * Constructor
         * @param node ros node reference
         * @param context global services container (timestamps, logging, etc.)
         */
        LidarVisualizer(rclcpp::Node::SharedPtr &node, Context &context) : node_{node}, context_{context} {}


        void drawPointCloudOnTopic(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &pc, const std::string &topic, const FrameType &frame);

        void drawApproximationOnTopic(const std::shared_ptr<std::vector<rtl::LineSegment3D<double>>> &ls, const std::string &topic, const FrameType &frame,
                                      visualization_msgs::msg::Marker::_color_type col);

        void drawLidarDetections(const std::vector<std::shared_ptr<DataModels::LidarDetection>> &detections, const std::string &topic, const FrameType &frame);

    private:

        rclcpp::Node::SharedPtr &node_;
        Context &context_;

        std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> pointCloudPublishers_;
        std::map<std::string, rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr> approximationsPublishers_;
        std::map<std::string, rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr> detectionsPublishers_;

        visualization_msgs::msg::MarkerArray
        lidarDetectionsToMarkerArray(const std::vector<std::shared_ptr<DataModels::LidarDetection>> &detections, const FrameType &frame);
    };
}
