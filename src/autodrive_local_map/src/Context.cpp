#include "Context.h"

namespace AutoDrive {

    Context Context::getEmptyContext() {
        LogService logger("", LogService::LogLevel::Off, false, false);
        LocalMap::TFTree tfTree("", logger);
        std::string emptyPath = "";
        return Context(logger, tfTree, emptyPath);
    }

    Context Context::getValidContext() {
        LogService logger("/tmp/context.log", LogService::LogLevel::Warning, true, true);
        LocalMap::TFTree tfTree("imu", logger);
        std::string emptyPath = "/home/autodrive/Developer/autodrive-localmap/src/autodrive_local_map/config.yaml";
        return Context(logger, tfTree, emptyPath);
    }

    Context::timePoint Context::getHighPrecisionTime() {
        auto time = std::chrono::high_resolution_clock::now();
        return time;
    }

    double Context::highPrecisionTimeToMilliseconds( std::chrono::duration<long, std::ratio<1, 1000000000>> t) {
        return static_cast<double>(t.count()) * 1e-6;
    }

}