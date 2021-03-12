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

#include <ros/ros.h>

#include <queue>
#include <visualization_msgs/Marker.h>

#include "Topics.h"
#include "data_models/local_map/LocalPosition.h"
#include "Context.h"
#include "VisualizationStructures.h"

namespace AtlasFusion::Visualizers {

    /**
     * Visualization backend (ROS) implementations for visualizing trajectory history as a polyline
     */
    class TrajectoryVisualizer {

    public:

        /**
         * Constructor
         * @param node ros node reference
         * @param context global services container (timestamps, logging, etc.)
         */
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

