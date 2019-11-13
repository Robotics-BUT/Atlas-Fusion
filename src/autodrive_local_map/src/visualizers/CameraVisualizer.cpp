#include "visualizers/CameraVisualizer.h"

#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "visualizers/Frames.h"

namespace AutoDrive::Visualizers {

    void CameraVisualizer::drawCameraLeftFrontImage(const std::shared_ptr<DataLoader::CameraFrameDataModel> data) const {
        drawRGBImage(data, cameraLeftFrontPublisher_, Frames::kCameraLeftFront);
    }


    void CameraVisualizer::drawCameraRightFrontImage(const std::shared_ptr<DataLoader::CameraFrameDataModel> data) const {
        drawRGBImage(data, cameraRightFrontPublisher_, Frames::kCameraRightFront);
    }


    void CameraVisualizer::drawCameraLeftSideImage(const std::shared_ptr<DataLoader::CameraFrameDataModel> data) const {
        drawRGBImage(data, cameraLeftSidePublisher_, Frames::kCameraLeftSide);
    }


    void CameraVisualizer::drawCameraRightSideImage(const std::shared_ptr<DataLoader::CameraFrameDataModel> data) const {
        drawRGBImage(data, cameraRightSidePublisher_, Frames::kCameraRightSide);
    }


    void CameraVisualizer::drawCameraIrImage(const std::shared_ptr<DataLoader::CameraIrFrameDataModel> data) const {
        cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
        cv_ptr->header.stamp = ros::Time::now();
        cv_ptr->header.frame_id = Frames::kCameraIr;
        cv_ptr->encoding = "mono8";
        cv_ptr->image = data->getImage();

        cameraIrPublisher_.publish(cv_ptr->toImageMsg());
    }


    void CameraVisualizer::drawRGBImage(const std::shared_ptr<DataLoader::CameraFrameDataModel> data, const image_transport::Publisher& publisher, const std::string& frame) const {

        cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
        cv_ptr->header.stamp = ros::Time::now();
        cv_ptr->header.frame_id = frame;
        cv_ptr->encoding = "bgr8";
        cv_ptr->image = data->getImage();

        publisher.publish(cv_ptr->toImageMsg());
    }
}