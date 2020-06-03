#pragma once

#include <chrono>
#include <ros/ros.h>

#include "LogService.h"
#include "local_map/TFTree.h"

namespace AutoDrive {

    /**
     *  Context is as container, that provides access to the global singleton instances that available for the
     *  entire project. There is an access to the global logging service, the sensor transformation tree and the
     *  calibration constants.
     */

    struct Context {

        using timePoint = std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>;

        explicit Context(LogService &logger, LocalMap::TFTree &tfTree, std::string& calibFolder)
                : logger_{logger}
                , tfTree_{tfTree}
                , calibFolder_{calibFolder}{

        }

        LogService &logger_;
        LocalMap::TFTree &tfTree_;
        const std::string& calibFolder_;

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
    };
}