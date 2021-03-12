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

#include "visualizers/TrajectoryVisualizer.h"

#include "local_map/Frames.h"

namespace AtlasFusion::Visualizers {


    void TrajectoryVisualizer::drawRawTrajectory(std::deque<DataModels::LocalPosition> data) const {
        Color c{0.0, 1.0, 0.0};
        drawTrajectory(data, rawTrajectoryPublisher_, c);
    }


    void TrajectoryVisualizer::drawFilteredTrajectory(std::deque<DataModels::LocalPosition> data) const {
        Color c{1.0, 0.0, 0.0};
        drawTrajectory(data, filteredTrajectoryPublisher_, c);
    }


    void TrajectoryVisualizer::drawImuGpsTrajectory(std::deque<DataModels::LocalPosition> data) const {
        Color c{0.0, 0.0, 1.0};
        drawTrajectory(data, imuGpsTrajectoryPublisher_, c);
    }

    void TrajectoryVisualizer::drawTrajectory(std::deque<DataModels::LocalPosition> data, const ros::Publisher& publisher, Color& color) const {
        visualization_msgs::Marker msg;

        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = LocalMap::Frames::kOrigin;

        msg.type = visualization_msgs::Marker::LINE_STRIP;
        msg.color.r = color.red_;
        msg.color.g = color.green_;
        msg.color.b = color.blue_;
        msg.color.a = color.alpha_;

        msg.scale.x = 0.02;

        while(!data.empty()) {
            auto p = data.front().getPosition();
            data.pop_front();

            geometry_msgs::Point point;
            point.x = p.x();
            point.y = p.y();
            point.z = p.z();

            msg.points.push_back(point);
        }

        publisher.publish(msg);
    }
}