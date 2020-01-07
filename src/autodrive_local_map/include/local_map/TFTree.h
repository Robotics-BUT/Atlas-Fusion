#pragma once
#include "LogService.h"
#include <rtl/Transformation3D.h>
#include <map>


namespace AutoDrive::LocalMap {

    class TFTree {

    public:

        TFTree(std::string rootFrameName, LogService& logger)
        : rootFrameName_(std::move(rootFrameName))
        , logger_(logger) {

        }

        void addFrame(rtl::Transformation3Dd tf, std::string name);
        rtl::Transformation3Dd getTransformationForFrame(const std::string& frameName);

        const std::vector<std::string> getFrameNames() {return frameNames_;};
        const std::string& getRootFrameName() {return rootFrameName_;};

        rtl::Vector3D<double> transformPointFromFrameToFrame(rtl::Vector3D<double>, const std::string& source, const std::string& destination);

    protected:

        std::string rootFrameName_;
        std::vector<std::string> frameNames_{};
        std::unordered_map<std::string, rtl::Transformation3Dd> frameMap_{};
        LogService& logger_;

        const std::unordered_map<std::string, rtl::Transformation3Dd>& getTree() {return frameMap_;};
    };

}