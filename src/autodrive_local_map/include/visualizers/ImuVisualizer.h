#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include "Topics.h"
#include "Context.h"
#include "data_models/imu/ImuImuDataModel.h"

namespace AutoDrive::Visualizers {


    class ImuVisualizer {

    public:

        ImuVisualizer(ros::NodeHandle& node, Context& context)
        : node_{node}
        , context_{context} {
        }

        void drawImuData(rtl::Vector3D<double> linAcc, std::string frame, std::string topic);

    protected:

        ros::NodeHandle& node_;
        Context& context_;

        std::map<std::string, std::shared_ptr<ros::Publisher>> publishers_;
    };

}