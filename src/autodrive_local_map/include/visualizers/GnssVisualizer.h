#pragma once

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include "Context.h"
#include "Topics.h"
#include "data_models/gnss/GnssPoseDataModel.h"

namespace AutoDrive::Visualizers {

    class GnssVisualizer {

    public:


        GnssVisualizer(ros::NodeHandle& node, Context& context)
        : node_{node}
        , context_{context} {
            gnssPublisher_ = node_.advertise<visualization_msgs::Marker>( Topics::kGnssTopic, 0 );
        }
        void drawGnssPose(const std::shared_ptr<DataModels::GnssPoseDataModel>) const;

    protected:
        ros::NodeHandle& node_;
        Context& context_;
        ros::Publisher gnssPublisher_;
    };

}
