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

#include "visualizers/FrustumVisualizer.h"
#include "util/IdentifierToFrameConversions.h"

namespace AutoDrive::Visualizers {

    void FrustumVisualizer::visualizeFrustumDetections(const std::vector<DataModels::FrustumDetection>& detections) {

        static size_t maxMarkerNo = 0;
        visualization_msgs::msg::MarkerArray msg;
        auto time = rclcpp::Time();

        size_t cnt = 0;
        for(const auto& detection : detections) {
            // Frustum
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = frameTypeName(FrameType::kImu);
            marker.header.stamp = time;
            marker.id = cnt++;
            marker.type = visualization_msgs::msg::Marker::LINE_LIST;
            marker.action = visualization_msgs::msg::Marker::ADD;

            marker.points = frustumToGeometryPointVector(detection.getFrustum());

            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.color = getColorByClass(detection.getClass());
            marker.color.a = 0.2;
            msg.markers.push_back(marker);
        }

        for(size_t i = cnt ; i < maxMarkerNo; i++) {
            visualization_msgs::msg::Marker marker;
            marker.id = i;
            marker.header.frame_id = frameTypeName(FrameType::kImu);
            marker.header.stamp = time;
            marker.type = visualization_msgs::msg::Marker::LINE_LIST;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.color.a = 0.0;
            msg.markers.push_back(marker);
        }
        frustumPublisher_->publish(msg);
        maxMarkerNo = std::max(maxMarkerNo, cnt);
    }

    void FrustumVisualizer::visualizeFusedFrustumDetections(const std::vector<DataModels::FrustumDetection>& detections) {

        static size_t maxMarkerNo = 0;
        visualization_msgs::msg::MarkerArray msg;
        auto time = rclcpp::Time();

        size_t cnt = 0;
        for(const auto& detection : detections) {
            // Frustum
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = frameTypeName(FrameType::kImu);
            marker.header.stamp = time;
            marker.id = cnt++;
            marker.type = visualization_msgs::msg::Marker::LINE_LIST;
            marker.action = visualization_msgs::msg::Marker::ADD;

            marker.points = frustumToGeometryPointVector(detection.getFrustum());

            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.color = getColorByClass(detection.getClass());
            msg.markers.push_back(marker);
        }

        for(size_t i = cnt ; i < maxMarkerNo; i++) {
            visualization_msgs::msg::Marker marker;
            marker.id = i;
            marker.header.frame_id = frameTypeName(FrameType::kImu);
            marker.header.stamp = time;
            marker.type = visualization_msgs::msg::Marker::LINE_LIST;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.color.a = 0.0;
            msg.markers.push_back(marker);
        }
        fusedFrustumPublisher_->publish(msg);
        maxMarkerNo = std::max(maxMarkerNo, cnt);
    }


    std::vector<geometry_msgs::msg::Point> FrustumVisualizer::frustumToGeometryPointVector(const std::shared_ptr<const rtl::Frustum3D<double>>& f) {

        geometry_msgs::msg::Point ntl;
        geometry_msgs::msg::Point ntr;
        geometry_msgs::msg::Point nbl;
        geometry_msgs::msg::Point nbr;

        geometry_msgs::msg::Point ftl;
        geometry_msgs::msg::Point ftr;
        geometry_msgs::msg::Point fbl;
        geometry_msgs::msg::Point fbr;

        ntl.x = f->getNearTopLeft().x();
        ntl.y = f->getNearTopLeft().y();
        ntl.z = f->getNearTopLeft().z();

        ntr.x = f->getNearTopRight().x();
        ntr.y = f->getNearTopRight().y();
        ntr.z = f->getNearTopRight().z();

        nbl.x = f->getNearBottomLeft().x();
        nbl.y = f->getNearBottomLeft().y();
        nbl.z = f->getNearBottomLeft().z();

        nbr.x = f->getNearBottomRight().x();
        nbr.y = f->getNearBottomRight().y();
        nbr.z = f->getNearBottomRight().z();

        ftl.x = f->getFarTopLeft().x();
        ftl.y = f->getFarTopLeft().y();
        ftl.z = f->getFarTopLeft().z();

        ftr.x = f->getFarTopRight().x();
        ftr.y = f->getFarTopRight().y();
        ftr.z = f->getFarTopRight().z();

        fbl.x = f->getFarBottomLeft().x();
        fbl.y = f->getFarBottomLeft().y();
        fbl.z = f->getFarBottomLeft().z();

        fbr.x = f->getFarBottomRight().x();
        fbr.y = f->getFarBottomRight().y();
        fbr.z = f->getFarBottomRight().z();

        return {nbl, nbr,
                nbr, ntr,
                ntr, ntl,
                ntl, nbl,
                fbl, fbr,
                fbr, ftr,
                ftr, ftl,
                ftl, fbl,
                ntl, ftl,
                ntr, ftr,
                nbl, fbl,
                nbr, fbr};
    }

    std_msgs::msg::ColorRGBA FrustumVisualizer::getColorByClass(DataModels::YoloDetectionClass cls) {
        std_msgs::msg::ColorRGBA output;
        output.a = 1.0;

        switch (getReducedDetectionClass(cls)) {
            case DataModels::ReducedYoloDetectionClasses::kPedestrian:  // red
                output.r = 1.0;
                output.g = 0.0;
                output.b = 0.0;
                break;
            case DataModels::ReducedYoloDetectionClasses::kBike:    // purple
                output.r = 1.0;
                output.g = 0.0;
                output.b = 1.0;
                break;
            case DataModels::ReducedYoloDetectionClasses::kVehicle: // blue
                output.r = 0.0;
                output.g = 0.0;
                output.b = 1.0;
                break;
            case DataModels::ReducedYoloDetectionClasses::kAnimal:  // green
                output.r = 0.0;
                output.g = 1.0;
                output.b = 0.0;
                break;
            case DataModels::ReducedYoloDetectionClasses::kTraffic:  // aqua
                output.r = 0.0;
                output.g = 1.0;
                output.b = 1.0;
                break;
            case DataModels::ReducedYoloDetectionClasses::kOther:   //yellow
                output.r = 1.0;
                output.g = 1.0;
                output.b = 0.0;
                break;
        }

        return output;
    }
}