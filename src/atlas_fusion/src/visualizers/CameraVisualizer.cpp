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

        checkCameraTopic(cameraTopic);
        checkCameraInfoTopic(cameraInfoTopic);

        auto ts = node_->get_clock()->now();

        std_msgs::msg::Header header;
        header.stamp = ts;
        header.frame_id = frameTypeName(frame);

        cameraPublishers_[cameraTopic].publish(toCameraMsg(data->getImage(), header, "bgr8"));
        publishCameraInfo(cameraParams_[frame], cameraInfoTopic, frame, ts);
    }


    void CameraVisualizer::drawIRCameraFrameWithTopic(const std::shared_ptr<DataModels::CameraIrFrameDataModel> &data, const std::string &cameraTopic,
                                                      const std::string &cameraInfoTopic,
                                                      const FrameType &frame) {

        checkCameraTopic(cameraTopic);
        checkCameraInfoTopic(cameraInfoTopic);

        auto ts = node_->get_clock()->now();

        std_msgs::msg::Header header;
        header.stamp = ts;
        header.frame_id = frameTypeName(FrameType::kCameraIr);

        cameraPublishers_[cameraTopic].publish(toCameraMsg(data->getImage(), header, "mono8"));
        publishCameraInfo(cameraParams_[frame], cameraInfoTopic, FrameType::kCameraIr, ts);
    }


    void CameraVisualizer::checkCameraTopic(const std::string &topic) {
        if (cameraPublishers_.count(topic) == 0) {
            cameraPublishers_[topic] = it_.advertise(topic, 0);
        }
    }


    void CameraVisualizer::checkCameraInfoTopic(const std::string &topic) {
        if (cameraInfoPublishers_.count(topic) == 0) {
            cameraInfoPublishers_[topic] = node_->create_publisher<sensor_msgs::msg::CameraInfo>(topic, 0);
        }
    }

    void CameraVisualizer::publishCameraInfo(const DataModels::CameraCalibrationParamsDataModel &params, const std::string &topic, const FrameType &frame,
                                             const rclcpp::Time &ts) {
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


    sensor_msgs::msg::Image CameraVisualizer::toCameraMsg(const cv::Mat &img, const std_msgs::msg::Header &header, const std::string &encoding) {
        sensor_msgs::msg::Image ros_image;
        ros_image.header = header;
        ros_image.height = img.rows;
        ros_image.width = img.cols;
        ros_image.encoding = encoding;
        ros_image.is_bigendian = (rcpputils::endian::native == rcpputils::endian::big);
        ros_image.step = img.cols * img.elemSize();
        size_t size = ros_image.step * img.rows;
        ros_image.data.resize(size);

        if (img.isContinuous()) {
            memcpy(reinterpret_cast<char *>(&ros_image.data[0]), img.data, size);
        } else {
            // Copy by row
            auto *ros_data_ptr = reinterpret_cast<uchar *>(&ros_image.data[0]);
            uchar *cv_data_ptr = img.data;
            for (int i = 0; i < img.rows; ++i) {
                memcpy(ros_data_ptr, cv_data_ptr, ros_image.step);
                ros_data_ptr += ros_image.step;
                cv_data_ptr += img.step;
            }
        }
        return ros_image;
    }

}