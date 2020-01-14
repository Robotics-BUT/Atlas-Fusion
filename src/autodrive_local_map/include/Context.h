#pragma once

#include <chrono>
#include <ros/ros.h>

#include "LogService.h"
#include "local_map/TFTree.h"

namespace AutoDrive {

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

        static Context getEmptyContext();
        static Context getValidContext();
        static timePoint getHighPrecisionTime();
        static double highPrecisionTimeToMilliseconds( std::chrono::duration<long, std::ratio<1, 1000000000>> t);
    };
}