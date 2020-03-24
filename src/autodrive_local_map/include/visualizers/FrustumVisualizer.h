#pragma once

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>

#include "data_models/local_map/FrustumDetection.h"
#include "data_models/yolo/YoloDetectionClass.h"
#include "rtl/Frustum.h"

#include "Context.h"
#include "Topics.h"

namespace AutoDrive::Visualizers {

    class FrustumVisualizer {

    public:

        FrustumVisualizer(ros::NodeHandle& node, Context& context)
        : node_{node}
        , context_{context} {

            frustumPublisher_ = node_.advertise<visualization_msgs::MarkerArray>( Topics::kYoloFrustumDetections, 0 );
        }


        void visualizeFrustumDetections(std::vector<std::shared_ptr<const DataModels::FrustumDetection>> detections);

    private:

        ros::NodeHandle& node_;
        Context& context_;

        ros::Publisher frustumPublisher_;

        std::vector<geometry_msgs::Point> frustumToGeometryPointVector(std::shared_ptr<const rtl::Frustum<double>> f);
        std::vector<geometry_msgs::Point> frustumToAxis(std::shared_ptr<const rtl::Frustum<double>> f);

        std_msgs::ColorRGBA getColorByClass(DataModels::YoloDetectionClass);
    };
}


