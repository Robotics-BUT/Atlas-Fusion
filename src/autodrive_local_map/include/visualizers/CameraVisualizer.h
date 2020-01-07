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

            cameraLeftFrontPublisher_ = it_.advertise( Topics::kCameraLeftFront, 0 );
            cameraLeftSidePublisher_ = it_.advertise( Topics::kCameraLeftSide, 0 );
            cameraRightFrontPublisher_ = it_.advertise( Topics::kCameraRightFront, 0 );
            cameraRightSidePublisher_ = it_.advertise( Topics::kCameraRightSide, 0 );
            cameraIrPublisher_ = it_.advertise( Topics::kCameraIr, 0 );

            cameraLeftFrontInfoPublisher_ = node_.advertise<sensor_msgs::CameraInfo>(Topics::kCameraLeftFrontInfo, 0);
            cameraLeftSideInfoPublisher_ = node_.advertise<sensor_msgs::CameraInfo>(Topics::kCameraLeftSideInfo, 0);
            cameraRightFrontInfoPublisher_ = node_.advertise<sensor_msgs::CameraInfo>(Topics::kCameraRightFrontInfo, 0);
            cameraRightSideInfoPublisher_ = node_.advertise<sensor_msgs::CameraInfo>(Topics::kCameraRightSideInfo, 0);
            cameraIrInfoPublisher_ = node_.advertise<sensor_msgs::CameraInfo>(Topics::kCameraIrInfo, 0);
        }

        void drawCameraLeftFrontImage(std::shared_ptr<DataModels::CameraFrameDataModel>) const;
        void drawCameraRightFrontImage(std::shared_ptr<DataModels::CameraFrameDataModel>) const;
        void drawCameraLeftSideImage(std::shared_ptr<DataModels::CameraFrameDataModel>) const;
        void drawCameraRightSideImage(std::shared_ptr<DataModels::CameraFrameDataModel>) const;
        void drawCameraIrImage(std::shared_ptr<DataModels::CameraIrFrameDataModel>) const;

        void setCameraParams(std::map<DataLoader::CameraIndentifier, std::shared_ptr<DataModels::CameraCalibrationParamsDataModel>>& cameraParams) {
            cameraParams_ = cameraParams;
        }

    private:

        ros::NodeHandle& node_;
        Context& context_;

        image_transport::Publisher cameraLeftFrontPublisher_;
        image_transport::Publisher cameraLeftSidePublisher_;
        image_transport::Publisher cameraRightFrontPublisher_;
        image_transport::Publisher cameraRightSidePublisher_;
        image_transport::Publisher cameraIrPublisher_;

        image_transport::ImageTransport it_;

        ros::Publisher cameraLeftFrontInfoPublisher_;
        ros::Publisher cameraLeftSideInfoPublisher_;
        ros::Publisher cameraRightFrontInfoPublisher_;
        ros::Publisher cameraRightSideInfoPublisher_;
        ros::Publisher cameraIrInfoPublisher_;

        std::map<DataLoader::CameraIndentifier, std::shared_ptr<DataModels::CameraCalibrationParamsDataModel>> cameraParams_;

        void drawRGBImage(std::shared_ptr<DataModels::CameraFrameDataModel> data, const image_transport::Publisher& publisher, const std::string& frame, ros::Time ts) const ;
        void publishCameraInfo(std::shared_ptr<DataModels::CameraCalibrationParamsDataModel> params, const ros::Publisher& pub, std::string, ros::Time ts) const;
    };

}
