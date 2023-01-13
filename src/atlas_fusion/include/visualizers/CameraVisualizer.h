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

#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/distortion_models.hpp"
#include "image_transport/image_transport.hpp"
#include "rcpputils/endian.hpp"

#include "Context.h"
#include "Topics.h"
#include "data_models/camera/CameraFrameDataModel.h"
#include "data_models/camera/CameraIrFrameDataModel.h"

namespace AutoDrive::Visualizers {

    /**
     * Visualization backend (ROS) implementations for visualizing camera data
     */
    class CameraVisualizer {

    public:

        /**
         * Constructor
         * @param node ros node reference
         * @param context global services container (timestamps, logging, etc.)
         */
        CameraVisualizer(rclcpp::Node::SharedPtr &node, Context &context) : node_{node}, context_{context}, it_(node_) {}

        void
        drawRGBCameraFrameWithTopic(const std::shared_ptr<DataModels::CameraFrameDataModel> &data, const std::string &cameraTopic,
                                    const std::string &cameraInfoTopic,
                                    const FrameType &frame);

        void drawIRCameraFrameWithTopic(const std::shared_ptr<DataModels::CameraIrFrameDataModel> &data, const std::string &topic,
                                        const std::string &cameraInfoTopic,
                                        const FrameType &frame);

        void setCameraParams(const FrameType &frame, DataModels::CameraCalibrationParamsDataModel cameraParams) {
            cameraParams_[frame] = std::move(cameraParams);
        }

    private:

        rclcpp::Node::SharedPtr &node_;
        Context &context_;

        image_transport::ImageTransport it_;

        std::map<std::string, image_transport::Publisher> cameraPublishers_;
        std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> cameraInfoPublishers_;

        std::map<FrameType, DataModels::CameraCalibrationParamsDataModel> cameraParams_;

        void checkCameraTopic(const std::string &);

        void checkCameraInfoTopic(const std::string &);

        void publishCameraInfo(const DataModels::CameraCalibrationParamsDataModel &params, const std::string &topic, const FrameType &frame, const rclcpp::Time& ts);

        sensor_msgs::msg::Image toCameraMsg(const cv::Mat& img, const std_msgs::msg::Header& header, const std::string& encoding);
    };

}
