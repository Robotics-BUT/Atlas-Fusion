#pragma once

#include <ros/ros.h>
#include "LogService.h"
#include "local_map/TFTree.h"

namespace AutoDrive {

    struct Context {

        explicit Context(LogService &logger, LocalMap::TFTree &tfTree, std::string& calibFolder)
                : logger_{logger}
                , tfTree_{tfTree}
                , calibFolder_{calibFolder}{

        }

        LogService &logger_;
        LocalMap::TFTree &tfTree_;
        const std::string& calibFolder_;

        static Context getEmptyContext() {
            LogService logger("", LogService::LogLevel::Off, false, false);
            LocalMap::TFTree tfTree("", logger);
            std::string emptyPath = "";
            return Context(logger, tfTree, emptyPath);
        }

        static Context getValidContext() {
            LogService logger("/tmp/context.log", LogService::LogLevel::Warning, true, true);
            LocalMap::TFTree tfTree("imu", logger);
            std::string emptyPath = "/home/autodrive/Developer/autodrive-localmap/src/autodrive_local_map/config.yaml";
            return Context(logger, tfTree, emptyPath);
        }
    };
}