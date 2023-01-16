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

#pragma once

#include "LogService.h"
#include "data_models/FrameTypes.h"


namespace AutoDrive::LocalMap {

    /**
     * Transformation Tree holds and allows simple searching in the transformation graph between the sensor frames, and
     * the world origin. Currently TF Tree supports only the root frame and 1 level of child-nodes and transformations
     * between them.
     */
    class TFTree {

    public:

        /**
         * Constructor
         * @param rootFrameName the central frame name
         * @param logger logger service instance
         */
        TFTree(FrameType rootFrameType, LogService &logger) : rootFrameType_(rootFrameType), logger_(logger) {}

        /**
         * Methods allows to add new child-frame under the root frame level
         * @param tf new child transformations
         * @param name new frame name
         */
        void addFrame(const rtl::RigidTf3D<double> &tf, const FrameType &frameType);

        /**
         * Method returns transformation between the root frame the the child frame
         * @param frameType child frame type
         * @return child transformation
         */
        rtl::RigidTf3D<double> getTransformationForFrame(const FrameType &frameType);

        /**
         * Method returns the vector of all child frame names.
         * @return all child frame names
         */
        std::vector<FrameType> getFrameTypes() const { return frameTypes_; };

        /**
         * Getter for root frame name
         * @return root frame name
         */
        const FrameType &getRootFrameType() const { return rootFrameType_; };

        /**
         * Method estimates transformation between two child frames.
         * @param srcPoint 3D point in the source coordinate systems (frame)
         * @param source source frame type
         * @param destination destination frame type
         * @return returns the point transformed from the original coordinate system to the new one.
         */
        rtl::Vector3D<double> transformPointFromFrameToFrame(const rtl::Vector3D<double> &, const FrameType &source, const FrameType &destination);

    protected:

        FrameType rootFrameType_;
        std::vector<FrameType> frameTypes_{};
        std::unordered_map<FrameType, rtl::RigidTf3D<double>> frameMap_{};
        LogService &logger_;

        const std::unordered_map<FrameType, rtl::RigidTf3D<double>> &getTree() { return frameMap_; };
    };

}