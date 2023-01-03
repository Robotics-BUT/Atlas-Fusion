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
#include "util/IdentifierToFrameConversions.h"

namespace AutoDrive::LocalMap {


    void TFTree::addFrame(const rtl::RigidTf3D<double>& tf, const FrameType& type) {

        if (frameMap_.find(type) != frameMap_.end()) {
            logger_.warning("Unable to insert " + frameTypeName(type) + " frame to TFTree. Frame already exists.");
            return;
        }

        frameMap_[type] = tf;
        frameTypes_.emplace_back(type);
    }


    rtl::RigidTf3D<double> TFTree::getTransformationForFrame(const FrameType& frameType) {
        if(frameType == rootFrameType_) {
            return rtl::RigidTf3D<double>{rtl::Quaternion<double>::identity(), {0.0, 0.0, 0.0}};
        }
        return frameMap_.at(frameType);
    }


    rtl::Vector3D<double> TFTree::transformPointFromFrameToFrame(const rtl::Vector3D<double>& srcPoint, const FrameType& source, const FrameType& destination) {
        auto tfs = getTransformationForFrame(source);
        auto tfd = getTransformationForFrame(destination);

        auto interResult = tfs(srcPoint);
        return tfd.inverted()(interResult);

    }

}