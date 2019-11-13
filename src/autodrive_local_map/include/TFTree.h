#pragma once
#include "LogService.h"
#include <rtl/Transformation3D.h>
#include <map>

namespace AutoDrive {

    class TFTree {

    public:

        TFTree(std::string rootFrameName, LogService& logger)
        : rootFrameName_(std::move(rootFrameName))
        , logger_(logger) {

        }

        void addFrame(rtl::Transformation3Dd tf, std::string name);

        const std::unordered_map<std::string, rtl::Transformation3Dd>& getTree() {return frameMap_;};
        const std::vector<std::string> getFrameNames() {return frameNames_;};
        const std::string& getRootFrameName() {return rootFrameName_;};

    protected:

        std::string rootFrameName_;
        std::vector<std::string> frameNames_{};
        std::unordered_map<std::string, rtl::Transformation3Dd> frameMap_{};
        LogService& logger_;
    };

}