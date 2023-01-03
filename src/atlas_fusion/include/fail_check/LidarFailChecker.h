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

#include "AbstractFailChecker.h"
#include "data_models/lidar/LidarScanDataModel.h"

namespace AutoDrive::FailCheck {

    /**
     * Validates LiDAR point cloud scans. Currently bypassed.
     */
    class LidarFailChecker : public AbstractFailChecker{

    public:

        LidarFailChecker() = delete;

        /**
         * Constructor
         * @param context container for global services (timestamps. logging, etc.)
         * @param selfModel self model of ego vehicle
         * @param environmentalModel model of environment current state
         */
        LidarFailChecker(Context &context, const Algorithms::SelfModel &selfModel, Algorithms::EnvironmentalModel &environmentalModel)
        : AbstractFailChecker{context, selfModel, environmentalModel} {}


        /**
         * Pipe to provide new sensor data into the LiDAR Fail Checker
         * @param data point cloud scan
         */
        void onNewData(std::shared_ptr<DataModels::LidarScanDataModel>);

    private:

    };
}

