#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include "Topics.h"
#include "LogService.h"
#include "data_loader/data_models/imu/ImuImuDataModel.h"

namespace AutoDrive::Visualizers {


    class ImuVisualizer {

    public:

        ImuVisualizer(ros::NodeHandle& node, LogService& logger)
        : node_(node)
        , logger_(logger) {
            imuPublisher_ = node_.advertise<sensor_msgs::Imu>( Topics::kImuTopic, 0 );
        }

        void drawImuData(const std::shared_ptr<DataLoader::ImuImuDataModel>) const;

    protected:

        ros::NodeHandle& node_;
        LogService& logger_;

        ros::Publisher imuPublisher_;

    };

}