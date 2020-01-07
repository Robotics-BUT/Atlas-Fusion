#include "visualizers/CameraVisualizer.h"

#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/distortion_models.h>
#include <camera_info_manager/camera_info_manager.h>

#include "local_map/Frames.h"

namespace AutoDrive::Visualizers {

    void CameraVisualizer::drawCameraLeftFrontImage(const std::shared_ptr<DataModels::CameraFrameDataModel> data) const {
        auto ts = ros::Time::now();
        publishCameraInfo(cameraParams_.at(DataLoader::CameraIndentifier::kCameraLeftFront), cameraLeftFrontInfoPublisher_, LocalMap::Frames::kCameraLeftFront, ts);
        drawRGBImage(data, cameraLeftFrontPublisher_, LocalMap::Frames::kCameraLeftFront, ts);
    }


    void CameraVisualizer::drawCameraRightFrontImage(const std::shared_ptr<DataModels::CameraFrameDataModel> data) const {
        auto ts = ros::Time::now();
        drawRGBImage(data, cameraRightFrontPublisher_, LocalMap::Frames::kCameraRightFront, ts);
        publishCameraInfo(cameraParams_.at(DataLoader::CameraIndentifier::kCameraRightFront), cameraRightFrontInfoPublisher_, LocalMap::Frames::kCameraRightFront, ts);
    }


    void CameraVisualizer::drawCameraLeftSideImage(const std::shared_ptr<DataModels::CameraFrameDataModel> data) const {
        auto ts = ros::Time::now();
        drawRGBImage(data, cameraLeftSidePublisher_, LocalMap::Frames::kCameraLeftSide, ts);
        publishCameraInfo(cameraParams_.at(DataLoader::CameraIndentifier::kCameraLeftSide), cameraLeftSideInfoPublisher_, LocalMap::Frames::kCameraLeftSide, ts);
    }


    void CameraVisualizer::drawCameraRightSideImage(const std::shared_ptr<DataModels::CameraFrameDataModel> data) const {
        auto ts = ros::Time::now();
        drawRGBImage(data, cameraRightSidePublisher_, LocalMap::Frames::kCameraRightSide, ts);
        publishCameraInfo(cameraParams_.at(DataLoader::CameraIndentifier::kCameraRightSide), cameraRightSideInfoPublisher_, LocalMap::Frames::kCameraRightSide, ts);
    }


    void CameraVisualizer::drawCameraIrImage(const std::shared_ptr<DataModels::CameraIrFrameDataModel> data) const {
        auto ts = ros::Time::now();

        std::shared_ptr<cv_bridge::CvImage> cv_ptr = std::make_shared<cv_bridge::CvImage>();
        cv_ptr->header.stamp = ts;
        cv_ptr->header.frame_id = LocalMap::Frames::kCameraIr;
        cv_ptr->encoding = "mono8";
        cv_ptr->image = data->getImage();

        cameraIrPublisher_.publish(cv_ptr->toImageMsg());

        publishCameraInfo(cameraParams_.at(DataLoader::CameraIndentifier::kCameraIr), cameraIrInfoPublisher_, LocalMap::Frames::kCameraIr, ts);
    }


    void CameraVisualizer::drawRGBImage(const std::shared_ptr<DataModels::CameraFrameDataModel> data, const image_transport::Publisher& publisher, const std::string& frame, ros::Time ts) const {

        cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
        cv_ptr->header.stamp = ts;
        cv_ptr->header.frame_id = frame;
        cv_ptr->encoding = "bgr8";
        cv_ptr->image = data->getImage();

        publisher.publish(cv_ptr->toImageMsg());
    }


    void CameraVisualizer::publishCameraInfo(std::shared_ptr<DataModels::CameraCalibrationParamsDataModel> params, const ros::Publisher& pub, std::string frame, ros::Time ts) const {
        sensor_msgs::CameraInfo msg;

        msg.header.stamp = ts;
        msg.header.frame_id = frame;

        msg.width = params->getWidth();
        msg.height = params->getHeight();

        msg.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

        auto dist = params->getDistortionParams();
        msg.D = {dist[0], dist[1], dist[2], dist[3], dist[4]};

        auto intrinsic = params->getIntrinsicParams();
        msg.K = {intrinsic[0][0], intrinsic[0][1], intrinsic[0][2],
                 intrinsic[1][0], intrinsic[1][1], intrinsic[1][2],
                 intrinsic[2][0], intrinsic[2][1], intrinsic[2][2],};

        msg.R = {1,0,0,
                 0,1,0,
                 0,0,1};


        msg.P = {intrinsic[0][0], intrinsic[0][1], intrinsic[0][2], 0,
                 intrinsic[1][0], intrinsic[1][1], intrinsic[1][2], 0,
                 intrinsic[2][0], intrinsic[2][1], intrinsic[2][2], 0};

        pub.publish(msg);
    }

}