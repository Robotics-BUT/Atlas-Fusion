#include "TFTree.h"

namespace AutoDrive {


    void TFTree::addFrame(rtl::Transformation3Dd tf, std::string name) {

        if (frameMap_.count(name) == 1) {
            logger_.warning("Unable to insert " + name + " frame to TFTree. Frame already exists.");
            return;
        }

        frameMap_[name] = tf;
        frameNames_.emplace_back(name);
    }
}