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

namespace AtlasFusion::Visualizers {

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
