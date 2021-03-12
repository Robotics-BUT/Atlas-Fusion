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
#include "YoloDetectionClass.h"

#include <iostream>

#include "data_models/GenericDataModel.h"

namespace AtlasFusion::DataModels{

    /**
     * Yolo Detection is a simple representation of a single object detection on the image data by the YOLO NN.
     */
    class YoloDetection {

    public:

        /**
         * 2D Bounding box representation
         */
        struct YoloDetectionBBox {
            YoloDetectionBBox (int x1, int y1, int x2, int y2)
                    : x1_(x1), x2_(x2), y1_(y1), y2_(y2) { }
            int x1_;
            int x2_;
            int y1_;
            int y2_;
        };

        /**
         * Constructor
         * @param x1 left boundary of the detection
         * @param y1 top boundary of the detection
         * @param x2 right  boundary of the detection
         * @param y2 bottom  boundary of the detection
         * @param detConfidence NN's detection confidence
         * @param classConfidence NN's confidence that the class have been identified correctly
         * @param cls detection class
         */
        explicit YoloDetection(int x1, int y1, int x2, int y2, float detConfidence, YoloDetectionClass cls)
        : bbox_{x1, y1, x2, y2}
        , detConfidence_(detConfidence)
        , detClass_(cls) {

        };

        /**
         * Getter for detection bounding box
         * @return bounding box of the detected object on the image
         */
        YoloDetectionBBox getBoundingBox() const { return bbox_; };

        /**
         * Getter for detection confidence
         * @return score of the NN's detection confidence
         */
        float getDetectionConfidence() const { return detConfidence_; };

        /**
         * Getter for most probable class of the detection
         * @return detection's class
         */
        YoloDetectionClass getDetectionClass() const { return detClass_; };

    private:

        YoloDetectionBBox bbox_;
        float detConfidence_;
        YoloDetectionClass detClass_;
    };
}