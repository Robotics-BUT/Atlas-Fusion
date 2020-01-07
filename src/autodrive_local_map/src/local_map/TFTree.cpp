#include "local_map/TFTree.h"

namespace AutoDrive::LocalMap {


    void TFTree::addFrame(rtl::Transformation3Dd tf, std::string name) {

        if (frameMap_.find(name) != frameMap_.end()) {
            logger_.warning("Unable to insert " + name + " frame to TFTree. Frame already exists.");
            return;
        }

        frameMap_[name] = tf;
        frameNames_.emplace_back(name);
    }


    rtl::Transformation3Dd TFTree::getTransformationForFrame(const std::string& frameName) {
        if(frameName == rootFrameName_) {
            return rtl::Transformation3Dd{};
        }
        return frameMap_.at(frameName);
    }


    rtl::Vector3D<double> TFTree::transformPointFromFrameToFrame(rtl::Vector3D<double> srcPoint, const std::string& source, const std::string& destination) {

        auto tfs = getTransformationForFrame(source);
        auto tfd = getTransformationForFrame(destination);

        auto interResult = tfs(srcPoint);
        return tfd.inverted()(interResult);

    }

}