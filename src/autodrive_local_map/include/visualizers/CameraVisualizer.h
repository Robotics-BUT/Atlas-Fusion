#pragma once

#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include "Context.h"
#include "Topics.h"
#include "data_models/camera/CameraFrameDataModel.h"
#include "data_models/camera/CameraIrFrameDataModel.h"

namespace AutoDrive::Visualizers {

    class CameraVisualizer {

    public:

        CameraVisualizer(ros::NodeHandle& node, Context& context)
        : node_{node}
        , context_{context}
        , it_(node_) {

        }

        void drawRGBCameraFrameWithTopic(std::shared_ptr<DataModels::CameraFrameDataModel> data, std::string cameraTopic, std::string cameraInfoTopic, std::string frame);
        void drawIRCameraFrameWithTopic(std::shared_ptr<DataModels::CameraIrFrameDataModel> data, std::string topic, std::string cameraInfoTopic, std::string frame);

        void setCameraParams(std::string frame, std::shared_ptr<DataModels::CameraCalibrationParamsDataModel> cameraParams) {
            cameraParams_[frame] = cameraParams;
        }

    private:

        ros::NodeHandle& node_;
        Context& context_;


        image_transport::ImageTransport it_;

        std::map<std::string, image_transport::Publisher> cameraPublishers_;
        std::map<std::string, ros::Publisher> cameraInfoPublishers_;

        std::map<std::string, std::shared_ptr<DataModels::CameraCalibrationParamsDataModel>> cameraParams_;

        void checkCameraTopic(std::string&);
        void checkCameraInfoTopic(std::string&);

        void publishCameraInfo(std::shared_ptr<DataModels::CameraCalibrationParamsDataModel> params, std::string& topic, std::string, ros::Time ts) ;
    };

}
