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

#include "data_models/all.h"
#include "local_map/Frames.h"

namespace AutoDrive {

    std::string frameTypeFromDataModel(const std::shared_ptr<DataModels::GenericDataModel> &dataModel) {
        auto type = dataModel->getType();
        switch (type) {
            case DataModels::DataModelTypes::kCameraDataModelType:
                switch (std::dynamic_pointer_cast<DataModels::CameraFrameDataModel>(dataModel)->getCameraIdentifier()) {
                    case DataLoader::CameraIndentifier::kCameraLeftFront:
                        return LocalMap::Frames::kCameraLeftFront;
                    case DataLoader::CameraIndentifier::kCameraLeftSide:
                        return LocalMap::Frames::kCameraLeftSide;
                    case DataLoader::CameraIndentifier::kCameraRightFront:
                        return LocalMap::Frames::kCameraRightFront;
                    case DataLoader::CameraIndentifier::kCameraRightSide:
                        return LocalMap::Frames::kCameraRightSide;
                    default:
                        throw std::runtime_error("Unable to convert between camera dataModel and frame type!");
                }

            case DataModels::DataModelTypes::kCameraIrDataModelType:
                return LocalMap::Frames::kCameraIr;

            case DataModels::DataModelTypes::kLidarScanDataModelType:
                switch (std::dynamic_pointer_cast<DataModels::LidarScanDataModel>(dataModel)->getLidarIdentifier()) {
                    case DataLoader::LidarIdentifier::kLeftLidar:
                        return LocalMap::Frames::kLidarLeft;
                    case DataLoader::LidarIdentifier::kRightLidar:
                        return LocalMap::Frames::kLidarRight;
                    case DataLoader::LidarIdentifier::kCenterLidar:
                        return LocalMap::Frames::kLidarCenter;
                    default:
                        throw std::runtime_error("Unable to convert between lidar dataModel and frame type!");
                }
            case DataModels::DataModelTypes::kRadarTiScanDataModelType:
                return LocalMap::Frames::kRadarTi;

            case DataModels::DataModelTypes::kImuDquatDataModelType:
            case DataModels::DataModelTypes::kImuGnssDataModelType:
            case DataModels::DataModelTypes::kImuImuDataModelType:
            case DataModels::DataModelTypes::kImuMagDataModelType:
            case DataModels::DataModelTypes::kImuPressDataModelType:
            case DataModels::DataModelTypes::kImuTimeDataModelType:
            case DataModels::DataModelTypes::kImuTempDataModelType:
                return LocalMap::Frames::kImuFrame;

            case DataModels::DataModelTypes::kGnssPositionDataModelType:
            case DataModels::DataModelTypes::kGnssTimeDataModelType:
                return LocalMap::Frames::kGnssAntennaRear;

            default:
                throw std::runtime_error("Unable to convert between sensor dataModel and frame type!");
        }
    }
}