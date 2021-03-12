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

#include <chrono>
#include <ros/ros.h>

#include "LogService.h"
#include "local_map/TFTree.h"
#include "FunctionalityFlags.h"

namespace AtlasFusion {

    /**
     *  Context is as container, that provides access to the global singleton instances that available for the
     *  entire project. There is an access to the global logging service, the sensor transformation tree and the
     *  calibration constants.
     */

    struct Context {

        using timePoint = std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>;

        explicit Context(LogService &logger, LocalMap::TFTree &tfTree, const std::string& calibFolder, const FunctionalityFlags& fFlags)
                : logger_{logger}
                , tfTree_{tfTree}
                , calibFolder_{calibFolder}
                , fFlags_{fFlags}{

        }

        LogService &logger_;
        LocalMap::TFTree &tfTree_;
        const std::string& calibFolder_;
        const FunctionalityFlags& fFlags_;


        /**
         * Method prepares the context that does not contains any usefull information. It is used only for the purpose
         * of the unit tests.
         * @return empty instance of the Context
         */
        static Context getEmptyContext();

        /**
         * Method gives current system time
         * @return system time
         */
        static timePoint getHighPrecisionTime();

        /**
        * Method gives current system time in milliseconds
        * @return system time in millsec
        */
        static double highPrecisionTimeToMilliseconds( std::chrono::duration<long, std::ratio<1, 1000000000>> t);

        /**
        * Method returns flags, that defines activated software functionalities
        * @return functionality flag class instance
        */
        const FunctionalityFlags& getFunctionalityFlags() const {return fFlags_;};
    };
}