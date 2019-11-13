#include "visualizers/ImuVisualizer.h"
#include "visualizers/Frames.h"

namespace AutoDrive::Visualizers {

    void ImuVisualizer::drawImuData(const std::shared_ptr<DataLoader::ImuImuDataModel> data) const {

        sensor_msgs::Imu msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = Frames::kImuFrame;
        msg.linear_acceleration.x = data->getLinearAcc().x();
        msg.linear_acceleration.y = data->getLinearAcc().y();
        msg.linear_acceleration.z = data->getLinearAcc().z();

        imuPublisher_.publish(msg);
    }
}