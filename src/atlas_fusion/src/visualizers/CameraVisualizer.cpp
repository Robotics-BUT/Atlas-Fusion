/*
 * Copyright 2020 Brno University of Technology
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom
 * the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT
 * OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
 * OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include "visualizers/CameraVisualizer.h"

#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/distortion_models.h>
#include <camera_info_manager/camera_info_manager.h>

#include "local_map/Frames.h"

namespace AtlasFusion::Visualizers {



    void CameraVisualizer::drawRGBCameraFrameWithTopic(std::shared_ptr<DataModels::CameraFrameDataModel> data, std::string cameraTopic, std::string cameraInfoTopic, std::string frame) {
        checkCameraTopic(cameraTopic);
        checkCameraInfoTopic(cameraInfoTopic);

        auto ts = ros::Time::now();

        cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
        cv_ptr->header.stamp = ts;
        cv_ptr->header.frame_id = frame;
        cv_ptr->encoding = "bgr8";
        cv_ptr->image = data->getImage();

        cameraPublishers_[cameraTopic].publish(cv_ptr->toImageMsg());
        publishCameraInfo(cameraParams_[frame], cameraInfoTopic, LocalMap::Frames::kCameraIr, ts);
    }


    void CameraVisualizer::drawIRCameraFrameWithTopic(std::shared_ptr<DataModels::CameraIrFrameDataModel> data, std::string cameraTopic, std::string cameraInfoTopic, std::string frame) {
        checkCameraTopic(cameraTopic);
        checkCameraInfoTopic(cameraInfoTopic);

        auto ts = ros::Time::now();

        std::shared_ptr<cv_bridge::CvImage> cv_ptr = std::make_shared<cv_bridge::CvImage>();
        cv_ptr->header.stamp = ts;
        cv_ptr->header.frame_id = LocalMap::Frames::kCameraIr;
        cv_ptr->encoding = "mono8";
        cv_ptr->image = data->getImage();

        cameraPublishers_[cameraTopic].publish(cv_ptr->toImageMsg());
        publishCameraInfo(cameraParams_[frame], cameraInfoTopic, LocalMap::Frames::kCameraIr, ts);

    }


    void CameraVisualizer::checkCameraTopic(std::string& topic) {
        if(cameraPublishers_.count(topic) == 0){
            cameraPublishers_[topic] = it_.advertise( topic, 0 );
        }
    }


    void CameraVisualizer::checkCameraInfoTopic(std::string& topic) {

        if(cameraInfoPublishers_.count(topic) == 0){
            cameraInfoPublishers_[topic] = node_.advertise<sensor_msgs::CameraInfo>( topic, 0 );
        }
    }


    void CameraVisualizer::publishCameraInfo(std::shared_ptr<DataModels::CameraCalibrationParamsDataModel> params, std::string& topic, std::string frame, ros::Time ts) {
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

        cameraInfoPublishers_[topic].publish(msg);
    }

}