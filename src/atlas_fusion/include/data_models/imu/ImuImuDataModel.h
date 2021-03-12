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

#include "data_models/GenericDataModel.h"
#include <rtl/Core.h>

namespace AtlasFusion::DataModels {

    /**
     * IMU-IMU Data Model represents the Xsens data packet that provides linear accelerations, angular velocities and
     * absolute orientiation, all in 3D
     */
    class ImuImuDataModel : public GenericDataModel {

    public:

        /**
         * Constructor
         * @param timestamp measurement timestamp
         * @param linAcc 3D Linear acceleration
         * @param angVel 3D angular velocities
         * @param orient absolute IMU orientation
         */
        ImuImuDataModel(uint64_t timestamp, rtl::Vector3D<double> linAcc, rtl::Vector3D<double> angVel, rtl::Quaternion<double> orient)
        : GenericDataModel(timestamp)
        , linearAcc_(linAcc)
        , angularVel_(angVel)
        , orientation_(orient) {
            type_ = DataModelTypes::kImuImuDataModelType;
        };

        std::string toString() override;

        /**
         * 3D Linear acceleration vector getter
         * @return 3D linear acceleration
         */
        rtl::Vector3D<double> getLinearAcc() {return linearAcc_;};

        /**
         * 3-axis angular velocities getter
         * @return 3D angular velocities
         */
        rtl::Vector3D<double> getAngularVel() {return angularVel_;};

        /**
         * Absolute IMU orientation getter
         * @return IMU orientation
         */
        rtl::Quaternion<double> getOrientation() {return orientation_;};

    private:

        rtl::Vector3D<double> linearAcc_;
        rtl::Vector3D<double> angularVel_;
        rtl::Quaternion<double> orientation_;


    };
}
