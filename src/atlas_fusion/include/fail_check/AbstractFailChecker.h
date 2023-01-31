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
#include "algorithms/SelfModel.h"
#include "algorithms/EnvironmentalModel.h"

namespace AutoDrive::FailCheck {

    struct SensorStatus {
        std::vector<float> statusVector;
        float status;
        std::string statusString;
    };

    /**
     * Abstract Fail Checker defines interface for other inherited Fail Checkers
     */
    class AbstractFailChecker {

    public:

        AbstractFailChecker() = delete;

        /**
         * Constructor
         * @param context container for global services (timestamps. logging, etc.)
         * @param selfModel self model of ego vehicle
         */
        AbstractFailChecker(Context &context, const Algorithms::SelfModel &selfModel, Algorithms::EnvironmentalModel &environmentalModel)
                : context_{context}, selfModel_{selfModel}, environmentalModel_{environmentalModel} {
            sensorStatus_.status = 1.0;
        }

        /**
        * Generates in-depth status struct
        * @return status struct
        */
        virtual SensorStatus getSensorStatus();

        virtual ~AbstractFailChecker() = default;

    protected:

        Context &context_;
        const Algorithms::SelfModel &selfModel_;
        Algorithms::EnvironmentalModel &environmentalModel_;
        SensorStatus sensorStatus_;
    };
}
