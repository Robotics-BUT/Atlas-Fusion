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

#include <rtl/Core.h>

#include "data_models/GenericDataModel.h"

namespace AtlasFusion::DataModels {

    /**
     * Imu Delta Quaterion represents the absolute orientation diff from last measurement
     */
    class ImuDquatDataModel : public GenericDataModel {

    public:

        /**
         * Constructor
         * @param timestamp measurement timestamp
         * @param dquat past orientation * dquat = current orientation
         */
        ImuDquatDataModel(const uint64_t timestamp, const rtl::Quaternion<double>& dquat)
        : GenericDataModel(timestamp)
        , dRotation_(dquat) {
            type_ = DataModelTypes::kImuDquatDataModelType;
        }

        std::string toString() override;

        /**
         * Orientation delta quaternion getter
         * @return delta quaternion
         */
        rtl::Quaternion<double> getDQuat() {return dRotation_;};

    private:

        rtl::Quaternion<double> dRotation_;

    };
}
