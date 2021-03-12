#pragma once

#include <ros/ros.h>
#include "Context.h"
#include "Topics.h"
#include "data_models/radar//RadarTiDataModel.h"

namespace AtlasFusion::Visualizers {

    class RadarVisualizer {

    public:
        /**
         * Constructor
         * @param node ros node reference
         * @param context global services container (timestamping, logging, etc.)
         */
        explicit RadarVisualizer(ros::NodeHandle &node, Context &context)
                : node_{node}
                , context_{context} {

        }

        void drawRadarDetectionsOnTopic(const std::vector<DataModels::RadarTiDataModel::Object>& objects, std::string topic, std::string frame);


    private:

        ros::NodeHandle& node_;
        Context& context_;

        std::map<std::string, std::shared_ptr<ros::Publisher>> publishers_;

        size_t radarTiMaxObjectVisCount_ = 0;
    };
}
