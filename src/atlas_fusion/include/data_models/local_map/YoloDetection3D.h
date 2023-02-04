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

#include "data_models/yolo/YoloDetection.h"

namespace AutoDrive::DataModels {

    /**
     * Simple 2D bounding box representation
     */
    struct BoundingBox2D {

        /**
         * Constructor
         * @param x1 left border
         * @param y1 top border
         * @param x2 right border
         * @param y2 bottom border
         */
        explicit BoundingBox2D(float x1, float y1, float x2, float y2)
                : x1_{x1}, y1_{y1}, x2_{x2}, y2_{y2} {}

        float x1_, y1_;
        float x2_, y2_;
    };

    /**
     * Yolo Detection 3D represents 2D detection with scene depth estimation
     */
    class YoloDetection3D {

    public:

        /**
         * Constructor
         * @param bb 2D bounding box at the camera's plain
         * @param distance estimated distance of the detected obstacle
         * @param detConfidence detection confidence
         * @param classConfidence confidence of the detected class
         * @param cls detection class
         */
        explicit YoloDetection3D(BoundingBox2D bb, float distance, float detConfidence, YoloDetectionClass cls)
                : bb_{bb}, distance_{distance}, detConfidence_{detConfidence}, cls_{cls} {

        }

        /**
         * 2D bounding box, that localize detection on the image getter
         * @return 2D bounding box
         */
        [[nodiscard]] BoundingBox2D getBoundingBox() const { return bb_; };

        /**
         * Estimated distance getter
         * @return estimated distance of the 2D detection
         */
        [[nodiscard]] float getDistance() const { return distance_; };

        /**
         * NN's confidence about the detection getter
         * @return confidence of the detection
         */
        [[nodiscard]] float getDetectionConfidence() const { return detConfidence_; };

        /**
         * Detection class getter
         * @return class of the detection
         */
        [[nodiscard]] YoloDetectionClass getClass() const { return cls_; };

    private:

        BoundingBox2D bb_;
        float distance_;
        float detConfidence_;
        YoloDetectionClass cls_;
    };
}
