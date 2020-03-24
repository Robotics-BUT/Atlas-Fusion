#pragma once
#include <memory>
#include <rtl/Frustum3D.h>

#include "data_models/yolo/YoloDetection.h"

namespace AutoDrive::DataModels {


    class FrustumDetection {

    public:

        explicit FrustumDetection(std::shared_ptr<const rtl::Frustum<double>> frustum, float detConfidence, float classConfidence, YoloDetectionClass cls)
                : frustum_{std::move(frustum)}
                , detConfidence_{detConfidence}
                , classConfidence_{classConfidence}
                , cls_{cls} {

        }

        std::shared_ptr<const rtl::Frustum<double>> getFrustum() const { return frustum_; };
        float getDetectionConfidence() const {return detConfidence_;};
        float getClassConfidence() const {return classConfidence_;};
        YoloDetectionClass getClass() const {return cls_;};

        void addParent(std::shared_ptr<DataModels::GenericDataModel> parent) {parents_.push_back(parent);};
        std::vector<std::shared_ptr<DataModels::GenericDataModel>> const getParents() {return parents_;};

    private:

        std::shared_ptr<const rtl::Frustum<double>> frustum_;
        float detConfidence_;
        float classConfidence_;
        YoloDetectionClass cls_;
        std::vector<std::shared_ptr<DataModels::GenericDataModel>> parents_;

    };
}
