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
     * 3D magnetic field indensity data model measured by IMU
     */
    class ImuMagDataModel : public GenericDataModel {

    public:

        /**
         * Constructor
         * @param timestamp measurement timestamp
         * @param mag 3D magnetic field intensity
         */
        ImuMagDataModel(uint64_t timestamp, rtl::Vector3D<double> mag)
        : GenericDataModel(timestamp)
        , mag_(mag){
            type_ = DataModelTypes::kImuMagDataModelType;
        };

        std::string toString() override;

        /**
         * 3D magnetic field intensity measurement getter
         * @return 3D vector of magnetic field intensity
         */
        rtl::Vector3D<double> getMagneticField() {return mag_;};

    private:

        rtl::Vector3D<double> mag_;

    };
}
