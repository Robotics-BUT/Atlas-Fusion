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

#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

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
        CameraVisualizer(ros::NodeHandle &node, Context &context)
                : node_{node}, context_{context}, it_(node_) {

        }

        void
        drawRGBCameraFrameWithTopic(const std::shared_ptr<DataModels::CameraFrameDataModel> &data, const std::string &cameraTopic, const std::string &cameraInfoTopic,
                                    const std::string &frame);

        void drawIRCameraFrameWithTopic(const std::shared_ptr<DataModels::CameraIrFrameDataModel> &data, const std::string &topic, const std::string &cameraInfoTopic,
                                        const std::string &frame);

        void setCameraParams(const std::string &frame, DataModels::CameraCalibrationParamsDataModel cameraParams) {
            cameraParams_[frame] = std::move(cameraParams);
        }

    private:

        ros::NodeHandle &node_;
        Context &context_;

        image_transport::ImageTransport it_;

        std::map<std::string, image_transport::Publisher> cameraPublishers_;
        std::map<std::string, ros::Publisher> cameraInfoPublishers_;

        std::map<std::string, DataModels::CameraCalibrationParamsDataModel> cameraParams_;

        void checkCameraTopic(const std::string &);

        void checkCameraInfoTopic(const std::string &);

        void publishCameraInfo(const DataModels::CameraCalibrationParamsDataModel &params, const std::string &topic, const std::string &frame, ros::Time ts);
    };

}
