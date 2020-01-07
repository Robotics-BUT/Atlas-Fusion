#pragma once
#include "YoloDetectionClass.h"

#include <iostream>

#include "data_models/GenericDataModel.h"

namespace AutoDrive::DataModels{

    class YoloDetection {

    public:

        struct YoloDetectionBBox {
            YoloDetectionBBox (size_t x1, size_t y1, size_t x2, size_t y2)
                    : x1_(x1), x2_(x2), y1_(y1), y2_(y2) { }
            size_t x1_;
            size_t x2_;
            size_t y1_;
            size_t y2_;
        };

        explicit YoloDetection(size_t x1, size_t y1, size_t x2, size_t y2, float detConfidence, float classConfidence, YoloDetectionClass cls)
        : bbox_{x1, y1, x2, y2}
        , detConfidence_(detConfidence)
        , classConfidence_(classConfidence)
        , detClass_(cls) {

        };

        YoloDetectionBBox getBoundingBox() { return bbox_; };
        float getDetectionConfidence() { return detConfidence_; };
        float getClassConfidence() { return classConfidence_; };
        YoloDetectionClass getDetectionClass() { return detClass_; };

    private:

        YoloDetectionBBox bbox_;
        float detConfidence_;
        float classConfidence_;
        YoloDetectionClass detClass_;
    };
}