#include "visualizers/TrajectoryVisualizer.h"

#include "local_map/Frames.h"

namespace AutoDrive::Visualizers {


    void TrajectoryVisualizer::drawRawTrajectory(std::deque<DataModels::LocalPosition> data) const {
        Color c{1.0, 0.0, 0.0};
        drawTrajectory(data, rawTrajectoryPublisher_, c);
    }


    void TrajectoryVisualizer::drawFilteredTrajectory(std::deque<DataModels::LocalPosition> data) const {
        Color c{0.0, 1.0, 0.0};
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