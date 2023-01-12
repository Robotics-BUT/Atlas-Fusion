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

#include "util/IdentifierToFrameConversions.h"

namespace AutoDrive::Visualizers {

    void CameraVisualizer::drawRGBCameraFrameWithTopic(const std::shared_ptr<DataModels::CameraFrameDataModel> &data, const std::string &cameraTopic,
                                                       const std::string &cameraInfoTopic,
                                                       const FrameType &frame) {
        /*
        checkCameraTopic(cameraTopic);
        checkCameraInfoTopic(cameraInfoTopic);

        auto ts = node_->get_clock()->now();

        std_msgs::msg::Header header;
        header.stamp = ts;
        header.frame_id = frameTypeName(frame);

        const cv::Mat& img = data->getImage();
        sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(header, "bgr8", img).toImageMsg();

        cameraPublishers_[cameraTopic]->publish(msg);
        publishCameraInfo(cameraParams_[frame], cameraInfoTopic, FrameType::kCameraIr, ts);
         */
    }


    void CameraVisualizer::drawIRCameraFrameWithTopic(const std::shared_ptr<DataModels::CameraIrFrameDataModel> &data, const std::string &cameraTopic,
                                                      const std::string &cameraInfoTopic,
                                                      const FrameType &frame) {
        /*
        checkCameraTopic(cameraTopic);
        checkCameraInfoTopic(cameraInfoTopic);

        auto ts = node_->get_clock()->now();

        std::shared_ptr<cv_bridge::CvImage> cv_ptr = std::make_shared<cv_bridge::CvImage>();
        cv_ptr->header.stamp = ts;
        cv_ptr->header.frame_id = frameTypeName(FrameType::kCameraIr);
        cv_ptr->encoding = "mono8";
        cv_ptr->image = data->getImage();

        cameraPublishers_[cameraTopic].publish(cv_ptr->toImageMsg());
        publishCameraInfo(cameraParams_[frame], cameraInfoTopic, FrameType::kCameraIr, ts);
         */
    }


    void CameraVisualizer::checkCameraTopic(const std::string &topic) {
        /*
        if (cameraPublishers_.count(topic) == 0) {
            cameraPublishers_[topic] = image_transport::create_publisher(node_, topic, 0);
        }
         */
    }


    void CameraVisualizer::checkCameraInfoTopic(const std::string &topic) {

        if (cameraInfoPublishers_.count(topic) == 0) {
            cameraInfoPublishers_[topic] = node_->create_publisher<sensor_msgs::msg::CameraInfo>(topic, 0);
        }
    }

    void CameraVisualizer::publishCameraInfo(const DataModels::CameraCalibrationParamsDataModel &params, const std::string &topic, const FrameType &frame,
                                             const rclcpp::Time& ts) {
        sensor_msgs::msg::CameraInfo msg;

        msg.header.stamp = ts;
        msg.header.frame_id = frameTypeName(frame);

        msg.width = params.getWidth();
        msg.height = params.getHeight();

        msg.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

        auto dist = params.getDistortionParams();
        msg.d = {dist[0], dist[1], dist[2], dist[3], dist[4]};

        auto intrinsic = params.getIntrinsicParams();
        msg.k = {intrinsic[0][0], intrinsic[0][1], intrinsic[0][2],
                 intrinsic[1][0], intrinsic[1][1], intrinsic[1][2],
                 intrinsic[2][0], intrinsic[2][1], intrinsic[2][2],};

        msg.r = {1, 0, 0,
                 0, 1, 0,
                 0, 0, 1};


        msg.p = {intrinsic[0][0], intrinsic[0][1], intrinsic[0][2], 0,
                 intrinsic[1][0], intrinsic[1][1], intrinsic[1][2], 0,
                 intrinsic[2][0], intrinsic[2][1], intrinsic[2][2], 0};

        cameraInfoPublishers_[topic]->publish(msg);
    }

}