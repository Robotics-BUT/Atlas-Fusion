#pragma once

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>

#include "data_models/local_map/FrustumDetection.h"
#include "data_models/yolo/YoloDetectionClass.h"
#include "rtl/Core.h"

#include "Context.h"
#include "Topics.h"

namespace AutoDrive::Visualizers {

    /**
     * Visualization backend (ROS) implementations for visualizing NN's frustum detections
     */
    class FrustumVisualizer {

    public:

        /**
         * Constructor
         * @param node ros node reference
         * @param context global services container (timestamps, logging, etc.)
         */
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

        std::vector<geometry_msgs::Point> frustumToGeometryPointVector(std::shared_ptr<const rtl::Frustum3D<double>> f);
        std::vector<geometry_msgs::Point> frustumToAxis(std::shared_ptr<const rtl::Frustum3D<double>> f);

        std_msgs::ColorRGBA getColorByClass(DataModels::YoloDetectionClass);
        std_msgs::ColorRGBA getEmptyColor();
    };
}


