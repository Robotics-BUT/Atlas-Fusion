#pragma once
#include <memory>
#include <rtl/Frustum3D.h>

#include "data_models/yolo/YoloDetection.h"

namespace AutoDrive::DataModels {


    /**
     * Frustum Detections merges the 3D frustum that represents the 3D structure in the map and the corresponding
     * NN's detection.
     */
    class FrustumDetection {

    public:

        /**
         * Constructor
         * @param frustum 3D space structure
         * @param detConfidence NN's confidence of the detection
         * @param classConfidence NN's confidence of the class of the detection
         * @param cls detection class
         */
        explicit FrustumDetection(std::shared_ptr<const rtl::Frustum<double>> frustum, float detConfidence, float classConfidence, YoloDetectionClass cls)
                : frustum_{std::move(frustum)}
                , detConfidence_{detConfidence}
                , classConfidence_{classConfidence}
                , cls_{cls} {

        }

        /**
         * Getter for frustum.
         * @return 3D space structure
         */
        std::shared_ptr<const rtl::Frustum<double>> getFrustum() const { return frustum_; };

        /**
         * NN's confidence of the detection getter
         * @return confidence of the detection
         */
        float getDetectionConfidence() const {return detConfidence_;};

        /**
         * NN's confidence of the detection class getter
         * @return confidence of the detected class
         */
        float getClassConfidence() const {return classConfidence_;};

        /**
         * Detection class getter
         * @return class of the detection
         */
        YoloDetectionClass getClass() const {return cls_;};

        /**
         * Inserts new reference into the vector of data model that preceded this data model
         * @param parent related preceded data model
         */
        void addParent(std::shared_ptr<DataModels::GenericDataModel> parent) {parents_.push_back(parent);};

        /**
         * Returns all the parents of this data model
         * @return vector of all parents of this data model.
         */
        std::vector<std::shared_ptr<DataModels::GenericDataModel>> const getParents() {return parents_;};

    private:

        std::shared_ptr<const rtl::Frustum<double>> frustum_;
        float detConfidence_;
        float classConfidence_;
        YoloDetectionClass cls_;
        std::vector<std::shared_ptr<DataModels::GenericDataModel>> parents_;

    };
}
