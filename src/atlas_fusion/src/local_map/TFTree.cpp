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

#include "local_map/TFTree.h"

namespace AtlasFusion::LocalMap {


    void TFTree::addFrame(rtl::RigidTf3D<double> tf, std::string name) {

        if (frameMap_.find(name) != frameMap_.end()) {
            logger_.warning("Unable to insert " + name + " frame to TFTree. Frame already exists.");
            return;
        }

        frameMap_[name] = tf;
        frameNames_.emplace_back(name);
    }


    rtl::RigidTf3D<double> TFTree::getTransformationForFrame(const std::string& frameName) {
        if(frameName == rootFrameName_) {
            return rtl::RigidTf3D<double>{rtl::Quaternion<double>::identity(), {0.0, 0.0, 0.0}};
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