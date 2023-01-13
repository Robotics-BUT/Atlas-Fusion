#pragma once

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "Context.h"
#include "Topics.h"
#include "data_models/radar/RadarTiDataModel.h"

namespace AutoDrive::Visualizers {

    class RadarVisualizer {

    public:
        /**
         * Constructor
         * @param node ros node reference
         * @param context global services container (timestamping, logging, etc.)
         */
        explicit RadarVisualizer(rclcpp::Node::SharedPtr &node, Context &context)
                : node_{node}, context_{context} {}

        void drawRadarDetectionsOnTopic(const std::vector<DataModels::RadarTiDataModel::Object> &objects, const std::string &topic, const FrameType &frame);


    private:

        rclcpp::Node::SharedPtr &node_;
        Context &context_;

        std::map<std::string, rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr> publishers_;

        size_t radarTiMaxObjectVisCount_ = 0;
    };
}
