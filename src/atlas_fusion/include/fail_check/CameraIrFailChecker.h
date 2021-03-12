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

#include "AbstrackFailChecker.h"
#include "data_models/camera/CameraIrFrameDataModel.h"

namespace AtlasFusion::FailCheck {

    /**
     * Validates IR camera frame data. Currently bypassed.
     */
    class CameraIrFailChecker : public AbstrackFailChecker{

    public:

        CameraIrFailChecker() = delete;

        /**
         * Constructor
         * @param context cantainer for global services (timestamps. logging, etc.)
         */
        CameraIrFailChecker(Context& context)
                : AbstrackFailChecker{context}
        {

        }

        /**
         * Pipe to provide new sensor data into the Camera RGB Fail Checker
         * @param data IR camera data frame
         */
        void onNewData(std::shared_ptr<DataModels::CameraIrFrameDataModel> data);

    private:

    };
}

