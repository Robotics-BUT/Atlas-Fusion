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

#include "visualizers/LidarVisualizer.h"
#include "util/IdentifierToFrameConversions.h"

#include "pcl_conversions/pcl_conversions.h"

namespace AutoDrive::Visualizers {

    void LidarVisualizer::drawPointCloudOnTopic(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &pc, const std::string& topic, const FrameType &frame) {

        if (pointCloudPublishers_.count(topic) == 0) {
            pointCloudPublishers_[topic] = node_->create_publisher<sensor_msgs::msg::PointCloud2>(topic, 0);
        }

        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*pc, msg);

        msg.header.stamp = node_->get_clock()->now();
        msg.header.frame_id = frameTypeName(frame);
        pointCloudPublishers_[topic]->publish(msg);
    }


    void LidarVisualizer::drawApproximationOnTopic(const std::shared_ptr<std::vector<rtl::LineSegment3D<double>>>& ls, const std::string& topic, const FrameType &frame,
                                                   visualization_msgs::msg::Marker::_color_type col) {
        if (ls->empty())
            return;

        if (approximationsPublishers_.count(topic) == 0) {
            approximationsPublishers_[topic] = node_->create_publisher<visualization_msgs::msg::Marker>(topic, 0);
        }

        visualization_msgs::msg::Marker lineSegments;
        lineSegments.header.frame_id = frameTypeName(frame);
        lineSegments.header.stamp = node_->get_clock()->now();
        lineSegments.type = visualization_msgs::msg::Marker::LINE_LIST;
        lineSegments.action = visualization_msgs::msg::Marker::ADD;

        for (auto &l: *ls) {
            if (l.beg().hasNaN() || l.end().hasNaN() || l.beg().lengthSquared() > 100000000 || l.end().lengthSquared() > 100000000)
                continue;
            geometry_msgs::msg::Point beg, end;
            beg.x = l.beg().x();
            beg.y = l.beg().y();
            beg.z = l.beg().z();
            lineSegments.points.push_back(beg);
            end.x = l.end().x();
            end.y = l.end().y();
            end.z = l.end().z();
            lineSegments.points.push_back(end);
        }

        lineSegments.scale.x = 0.05;
        lineSegments.scale.y = 0.05;
        lineSegments.scale.z = 0.05;

        lineSegments.color = col;

        approximationsPublishers_[topic]->publish(lineSegments);
    }


    void LidarVisualizer::drawLidarDetections(const std::vector<std::shared_ptr<DataModels::LidarDetection>>& detections, const std::string& topic, const FrameType &frame) {

        if (detectionsPublishers_.count(topic) == 0) {
            detectionsPublishers_[topic] = node_->create_publisher<visualization_msgs::msg::MarkerArray>(topic, 0);
        }
        detectionsPublishers_[topic]->publish(lidarDetectionsToMarkerArray(detections, frame));
    }


    visualization_msgs::msg::MarkerArray
    LidarVisualizer::lidarDetectionsToMarkerArray(const std::vector<std::shared_ptr<DataModels::LidarDetection>>& detections, const FrameType &frame) {

        visualization_msgs::msg::MarkerArray output;

        auto timestamp = node_->get_clock()->now();

        static size_t maxMarkerNo = 0;
        size_t cnt = 0;
        for (const auto &detection: detections) {

            double dx = detection->getBoundingBox().max().getElement(0) - detection->getBoundingBox().min().getElement(0);
            double dy = detection->getBoundingBox().max().getElement(1) - detection->getBoundingBox().min().getElement(1);
            double dz = detection->getBoundingBox().max().getElement(2) - detection->getBoundingBox().min().getElement(2);

            double cx = (detection->getBoundingBox().max().getElement(0) + detection->getBoundingBox().min().getElement(0)) / 2;
            double cy = (detection->getBoundingBox().max().getElement(1) + detection->getBoundingBox().min().getElement(1)) / 2;
            double cz = (detection->getBoundingBox().max().getElement(2) + detection->getBoundingBox().min().getElement(2)) / 2;

            // Bounding Box
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = frameTypeName(frame);
            marker.header.stamp = timestamp;
            marker.id = cnt++;
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.action = visualization_msgs::msg::Marker::ADD;

            marker.pose.position.x = cx;
            marker.pose.position.y = cy;
            marker.pose.position.z = cz;

            marker.pose.orientation.x = detection->getOrientation().x();
            marker.pose.orientation.y = detection->getOrientation().y();
            marker.pose.orientation.z = detection->getOrientation().z();
            marker.pose.orientation.w = detection->getOrientation().w();

            marker.scale.x = dx;
            marker.scale.y = dy;
            marker.scale.z = dz;

            marker.color.a = 0.5;
            if(detection->getDetectionClass() == DataModels::YoloDetectionClass::kCar) {
                marker.color.r = 0.0;
                marker.color.g = 0.0;
                marker.color.b = 1.0;
            } else if(detection->getDetectionClass() == DataModels::YoloDetectionClass::kPerson)  {
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
            } else {
                marker.color.r = 0.7;
                marker.color.g = 0.7;
                marker.color.b = 0.7;
            }

            output.markers.push_back(marker);

            visualization_msgs::msg::Marker text;
            text.header.frame_id = frameTypeName(frame);
            text.header.stamp = timestamp;
            text.id = cnt++;
            text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text.action = visualization_msgs::msg::Marker::ADD;

            text.pose.position.x = detection->getBoundingBox().max().getElement(0);
            text.pose.position.y = detection->getBoundingBox().max().getElement(1);
            text.pose.position.z = detection->getBoundingBox().max().getElement(2);

            std::stringstream sstream;
            sstream << "ID: " << detection->getID() << std::endl
                    << "TTL: " << detection->getTTL() << std::endl;
            text.text = sstream.str();

            text.scale.x = 0.3;
            text.scale.y = 0.3;
            text.scale.z = 0.3;

            text.color.a = 0.0;
            text.color.r = 0.7;
            text.color.g = 0.7;
            text.color.b = 0.7;

            output.markers.push_back(text);
        }

        for(size_t i = cnt ; i < maxMarkerNo; i++) {
            visualization_msgs::msg::Marker marker;
            marker.id = i;
            marker.header.frame_id = frameTypeName(FrameType::kImu);
            marker.header.stamp = timestamp;
            marker.type = visualization_msgs::msg::Marker::LINE_LIST;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.color.a = 0.0;
            output.markers.push_back(marker);
        }

        maxMarkerNo = std::max(maxMarkerNo, cnt);

        return output;
    }
}