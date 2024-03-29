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

# pragma once

#include "Context.h"
#include "data_models/GenericDataModel.h"

namespace AutoDrive::FailCheck {

    /**
     * Abstract Fail Checker defines interface for other inherited Fail Checkers
     */
    class AbstrackFailChecker {

    public:

        AbstrackFailChecker() = delete;

        /**
         * Constructor
         * @param context cantainer for global services (timestamps. logging, etc.)
         */
        explicit AbstrackFailChecker(Context& context) :
        context_{context}
        {
            sensorStatus_ = 1.0;
        }

        /**
         * Reports reliability of the sensor
         * @return
         */
        virtual float getSensorStatus();

    protected:

        Context& context_;
        float sensorStatus_;

    };
}
