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

#include "fail_check/FailChecker.h"

namespace AutoDrive::FailCheck {


    void FailChecker::onNewData(const std::shared_ptr<DataModels::GenericDataModel> &data, const FrameType &sensorID) {

        if (failCheckers_.count(sensorID) == 0) {
            context_.logger_.warning("Unable to find fail checker for sensor");
            return;
        }

        std::shared_ptr<AbstractFailChecker> sensor;
        switch (data->getType()) {
            case DataModels::DataModelTypes::kCameraDataModelType:
                sensor = failCheckers_[sensorID];
                std::dynamic_pointer_cast<CameraRGBFailChecker>(sensor)->onNewData(std::dynamic_pointer_cast<DataModels::CameraFrameDataModel>(data));
                break;


            case DataModels::DataModelTypes::kCameraIrDataModelType:
                sensor = failCheckers_[sensorID];
                std::dynamic_pointer_cast<CameraIrFailChecker>(sensor)->onNewData(std::dynamic_pointer_cast<DataModels::CameraIrFrameDataModel>(data));
                break;


            case DataModels::DataModelTypes::kLidarScanDataModelType:
                sensor = failCheckers_[sensorID];
                std::dynamic_pointer_cast<LidarFailChecker>(sensor)->onNewData(std::dynamic_pointer_cast<DataModels::LidarScanDataModel>(data));
                break;

            case DataModels::DataModelTypes::kRadarTiScanDataModelType:
                sensor = failCheckers_[sensorID];
                std::dynamic_pointer_cast<RadarTiFailChecker>(sensor)->onNewData(std::dynamic_pointer_cast<DataModels::RadarTiDataModel>(data));
                break;


            case DataModels::DataModelTypes::kGnssPositionDataModelType:
                sensor = failCheckers_[sensorID];
                std::dynamic_pointer_cast<GnssFailChecker>(sensor)->onNewData(std::dynamic_pointer_cast<DataModels::GnssPoseDataModel>(data));
                break;
            case DataModels::DataModelTypes::kGnssTimeDataModelType:
                sensor = failCheckers_[sensorID];
                std::dynamic_pointer_cast<GnssFailChecker>(sensor)->onNewData(std::dynamic_pointer_cast<DataModels::GnssTimeDataModel>(data));
                break;


            case DataModels::DataModelTypes::kImuDquatDataModelType:
                sensor = failCheckers_[sensorID];
                std::dynamic_pointer_cast<ImuFailChecker>(sensor)->onNewData(std::dynamic_pointer_cast<DataModels::ImuDquatDataModel>(data));
                break;
            case DataModels::DataModelTypes::kImuGnssDataModelType:
                sensor = failCheckers_[sensorID];
                std::dynamic_pointer_cast<ImuFailChecker>(sensor)->onNewData(std::dynamic_pointer_cast<DataModels::ImuGnssDataModel>(data));
                break;
            case DataModels::DataModelTypes::kImuImuDataModelType:
                sensor = failCheckers_[sensorID];
                std::dynamic_pointer_cast<ImuFailChecker>(sensor)->onNewData(std::dynamic_pointer_cast<DataModels::ImuImuDataModel>(data));
                break;
            case DataModels::DataModelTypes::kImuMagDataModelType:
                sensor = failCheckers_[sensorID];
                std::dynamic_pointer_cast<ImuFailChecker>(sensor)->onNewData(std::dynamic_pointer_cast<DataModels::ImuMagDataModel>(data));
                break;
            case DataModels::DataModelTypes::kImuPressDataModelType:
                sensor = failCheckers_[sensorID];
                std::dynamic_pointer_cast<ImuFailChecker>(sensor)->onNewData(std::dynamic_pointer_cast<DataModels::ImuPressureDataModel>(data));
                break;
            case DataModels::DataModelTypes::kImuTempDataModelType:
                sensor = failCheckers_[sensorID];
                std::dynamic_pointer_cast<ImuFailChecker>(sensor)->onNewData(std::dynamic_pointer_cast<DataModels::ImuTempDataModel>(data));
                break;
            case DataModels::DataModelTypes::kImuTimeDataModelType:
                sensor = failCheckers_[sensorID];
                std::dynamic_pointer_cast<ImuFailChecker>(sensor)->onNewData(std::dynamic_pointer_cast<DataModels::ImuTimeDataModel>(data));
                break;

            default:
                context_.logger_.warning("Unindentified data type during FailCheck processing");
                break;
        }
    }


    float FailChecker::getSensorStatus(const FrameType &sensor) {

        if (failCheckers_.count(sensor) == 0) {
            context_.logger_.warning("Unable to find fail checker for sensor");
            return 0;
        }

        return failCheckers_[sensor]->getSensorStatus();
    }

    std::string FailChecker::getSensorStatusString(const FrameType &sensor) {
        if (failCheckers_.count(sensor) == 0) {
            context_.logger_.warning("Unable to find fail checker for sensor");
            return 0;
        }

        return failCheckers_[sensor]->getSensorStatusString();
    }
}