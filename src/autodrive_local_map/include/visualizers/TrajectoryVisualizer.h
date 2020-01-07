#pragma once

#include <ros/ros.h>

#include <queue>
#include <visualization_msgs/Marker.h>

#include "Topics.h"
#include "data_models/local_map/LocalPosition.h"
#include "Context.h"
#include "VisualizationStructures.h"

namespace AutoDrive::Visualizers {

    class TrajectoryVisualizer {

    public:

        TrajectoryVisualizer(ros::NodeHandle& node, Context& context)
        : node_{node}
        , context_{context} {
            rawTrajectoryPublisher_ = node_.advertise<visualization_msgs::Marker>( Topics::kRawTrajectory, 0 );
            filteredTrajectoryPublisher_ = node_.advertise<visualization_msgs::Marker>( Topics::kFilteredTrajectory, 0 );
            imuGpsTrajectoryPublisher_ = node_.advertise<visualization_msgs::Marker>( Topics::kImuGpsTrajectory, 0 );
        }

        void drawRawTrajectory(std::deque<DataModels::LocalPosition> data) const;
        void drawFilteredTrajectory(std::deque<DataModels::LocalPosition> data) const;
        void drawImuGpsTrajectory(std::deque<DataModels::LocalPosition> data) const;

    protected:

        ros::NodeHandle& node_;
        Context& context_;
        ros::Publisher rawTrajectoryPublisher_;
        ros::Publisher filteredTrajectoryPublisher_;
        ros::Publisher imuGpsTrajectoryPublisher_;

        void drawTrajectory(std::deque<DataModels::LocalPosition> data, const ros::Publisher& publisher, Color& color) const;

    };

}

