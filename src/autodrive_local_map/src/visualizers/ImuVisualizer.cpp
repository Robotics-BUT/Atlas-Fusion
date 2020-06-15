#include "visualizers/ImuVisualizer.h"
#include "local_map/Frames.h"

namespace AutoDrive::Visualizers {

    void ImuVisualizer::drawImuData(const rtl::Vector3D<double> linAcc, std::string /*frame*/, std::string topic) {

        if(publishers_.count(topic) == 0) {
            publishers_[topic] = std::make_shared<ros::Publisher>(node_.advertise<sensor_msgs::Imu>( topic, 0));
        }

        sensor_msgs::Imu msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = LocalMap::Frames::kImuFrame;
        msg.linear_acceleration.x = linAcc.x();
        msg.linear_acceleration.y = linAcc.y();
        msg.linear_acceleration.z = linAcc.z();

        publishers_[topic]->publish(msg);
    }
}