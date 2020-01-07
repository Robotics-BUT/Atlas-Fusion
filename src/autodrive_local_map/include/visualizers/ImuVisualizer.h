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
            imuPublisher_ = node_.advertise<sensor_msgs::Imu>( Topics::kImuTopic, 0 );
        }

        void drawImuData(rtl::Vector3D<double> linAcc) const;

    protected:

        ros::NodeHandle& node_;
        Context& context_;
        ros::Publisher imuPublisher_;
    };

}