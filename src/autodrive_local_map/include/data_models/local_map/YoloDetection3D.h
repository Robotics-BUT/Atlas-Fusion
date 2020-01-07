#pragma once
#include "data_models/yolo/YoloDetection.h"
#include <memory>

namespace AutoDrive::DataModels {

    struct BoundingBox2D{

        explicit BoundingBox2D(float x1, float y1, float x2, float y2)
        : x1_{x1}, y1_{y1}, x2_{x2}, y2_{y2} {}

        float x1_, y1_;
        float x2_, y2_;
    };

    class YoloDetection3D {

    public:

        explicit YoloDetection3D(BoundingBox2D bb, float distance, float detConfidence, float classConfidence, YoloDetectionClass cls)
        : bb_{bb}
        , distance_{distance}
        , detConfidence_{detConfidence}
        , classConfidence_{classConfidence}
        , cls_{cls} {

        }

        BoundingBox2D getBoundingBox() const {return bb_;};
        float getDistance() const {return distance_;};
        float getDetectionConfidence() const {return detConfidence_;};
        float getClassConfidence() const {return classConfidence_;};
        YoloDetectionClass getClass() const {return cls_;};

        void addParent(std::shared_ptr<DataModels::GenericDataModel> parent) {parents_.push_back(parent);};
        std::vector<std::shared_ptr<DataModels::GenericDataModel>> const getParents() {return parents_;};

    private:

        BoundingBox2D bb_;
        float distance_;
        float detConfidence_;
        float classConfidence_;
        YoloDetectionClass cls_;
        std::vector<std::shared_ptr<DataModels::GenericDataModel>> parents_;

    };
}
