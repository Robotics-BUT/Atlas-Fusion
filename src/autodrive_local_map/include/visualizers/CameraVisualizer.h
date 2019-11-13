#pragma once

#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>

#include "LogService.h"
#include "Topics.h"
#include "data_loader/data_models/camera/CameraFrameDataModel.h"
#include "data_loader/data_models/camera/CameraIrFrameDataModel.h"

namespace AutoDrive::Visualizers {

    class CameraVisualizer {

    public:

        CameraVisualizer(ros::NodeHandle& node, LogService& logger)
        : node_(node)
        , logger_(logger)
        , it_(node) {

            cameraLeftFrontPublisher_ = it_.advertise( Topics::kCameraLeftFront, 0 );
            cameraLeftSidePublisher_ = it_.advertise( Topics::kCameraLeftSide, 0 );
            cameraRightFrontPublisher_ = it_.advertise( Topics::kCameraRightFront, 0 );
            cameraRightSidePublisher_ = it_.advertise( Topics::kCameraRightSide, 0 );

            cameraIrPublisher_ = it_.advertise( Topics::kCameraIr, 0 );



        }

        void drawCameraLeftFrontImage(const std::shared_ptr<DataLoader::CameraFrameDataModel>) const;
        void drawCameraRightFrontImage(const std::shared_ptr<DataLoader::CameraFrameDataModel>) const;
        void drawCameraLeftSideImage(const std::shared_ptr<DataLoader::CameraFrameDataModel>) const;
        void drawCameraRightSideImage(const std::shared_ptr<DataLoader::CameraFrameDataModel>) const;
        void drawCameraIrImage(const std::shared_ptr<DataLoader::CameraIrFrameDataModel>) const;

    private:

        ros::NodeHandle& node_;
        LogService& logger_;

        image_transport::Publisher cameraLeftFrontPublisher_;
        image_transport::Publisher cameraLeftSidePublisher_;
        image_transport::Publisher cameraRightFrontPublisher_;
        image_transport::Publisher cameraRightSidePublisher_;

        image_transport::Publisher cameraIrPublisher_;

        image_transport::ImageTransport it_;

        void drawRGBImage(const std::shared_ptr<DataLoader::CameraFrameDataModel> data, const image_transport::Publisher& publisher, const std::string& frame) const ;
    };

}
