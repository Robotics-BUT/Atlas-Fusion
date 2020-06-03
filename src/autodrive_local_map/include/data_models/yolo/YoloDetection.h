#pragma once
#include "YoloDetectionClass.h"

#include <iostream>

#include "data_models/GenericDataModel.h"

namespace AutoDrive::DataModels{

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
        explicit YoloDetection(int x1, int y1, int x2, int y2, float detConfidence, float classConfidence, YoloDetectionClass cls)
        : bbox_{x1, y1, x2, y2}
        , detConfidence_(detConfidence)
        , classConfidence_(classConfidence)
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
         * Getter for detection class confidence
         * @return score of the NN's detection class confidence
         */
        float getClassConfidence() const { return classConfidence_; };

        /**
         * Getter for most probable class of the detection
         * @return detection's class
         */
        YoloDetectionClass getDetectionClass() const { return detClass_; };

    private:

        YoloDetectionBBox bbox_;
        float detConfidence_;
        float classConfidence_;
        YoloDetectionClass detClass_;
    };
}