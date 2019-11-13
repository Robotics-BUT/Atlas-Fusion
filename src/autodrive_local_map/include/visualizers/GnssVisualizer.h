#pragma once

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include "LogService.h"
#include "Topics.h"
#include "data_loader/data_models/gnss/GnssPoseDataModel.h"

namespace AutoDrive::Visualizers {

    class GnssVisualizer {

    public:


        GnssVisualizer(ros::NodeHandle& node, LogService& logger)
        : node_(node)
        , logger_(logger) {
            gnssPublisher_ = node_.advertise<visualization_msgs::Marker>( Topics::kGnssTopic, 0 );
        }

        void drawGnssPose(const std::shared_ptr<DataLoader::GnssPoseDataModel>) const;

    protected:

        ros::NodeHandle& node_;
        LogService& logger_;

        ros::Publisher gnssPublisher_;

    };

}
