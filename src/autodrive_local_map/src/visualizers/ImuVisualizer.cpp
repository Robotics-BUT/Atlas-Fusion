#include "visualizers/ImuVisualizer.h"
#include "local_map/Frames.h"

namespace AutoDrive::Visualizers {

    void ImuVisualizer::drawImuData(const rtl::Vector3D<double> linAcc) const {

        sensor_msgs::Imu msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = LocalMap::Frames::kImuFrame;
        msg.linear_acceleration.x = linAcc.x();
        msg.linear_acceleration.y = linAcc.y();
        msg.linear_acceleration.z = linAcc.z();

        imuPublisher_.publish(msg);
    }
}