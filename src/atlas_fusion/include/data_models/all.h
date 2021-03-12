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

#include "data_models/yolo/YoloDetection.h"
#include "data_models/yolo/YoloDetectionClass.h"

#include "data_models/gnss/GnssTimeDataModel.h"
#include "data_models/gnss/GnssPoseDataModel.h"

#include "data_models/imu/ImuDquatDataModel.h"
#include "data_models/imu/ImuGnssDataModel.h"
#include "data_models/imu/ImuImuDataModel.h"
#include "data_models/imu/ImuMagDataModel.h"
#include "data_models/imu/ImuPressureDataModel.h"
#include "data_models/imu/ImuTempDataModel.h"
#include "data_models/imu/ImuTimeDataModel.h"

#include "data_models/lidar/LidarScanDataModel.h"
#include "data_models/radar/RadarTiDataModel.h"

#include "data_models/camera/CameraFrameDataModel.h"
#include "data_models/camera/CameraIrFrameDataModel.h"
#include "data_models/camera/DepthMapDataModel.h"
#include "data_models/camera/CameraCalibrationParamsDataModel.h"

#include "ErrorDataModel.h"
#include "GenericDataModel.h"