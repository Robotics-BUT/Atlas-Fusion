#pragma once
#include "data_models/yolo/YoloDetection.h"
#include <memory>

namespace AutoDrive::DataModels {

    /**
     * Simple 2D bounding box representation
     */
    struct BoundingBox2D{

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
         * @param classConfidence confidence of the detected dlass
         * @param cls detection class
         */
        explicit YoloDetection3D(BoundingBox2D bb, float distance, float detConfidence, float classConfidence, YoloDetectionClass cls)
        : bb_{bb}
        , distance_{distance}
        , detConfidence_{detConfidence}
        , classConfidence_{classConfidence}
        , cls_{cls} {

        }

        /**
         * 2D bounding box, taht localize detection on the image getter
         * @return 2D bounding box
         */
        BoundingBox2D getBoundingBox() const {return bb_;};

        /**
         * Estimated distance getter
         * @return estimated distance of the 2D detection
         */
        float getDistance() const {return distance_;};

        /**
         * NN's confidence aobut the detection getter
         * @return confidence of the detection
         */
        float getDetectionConfidence() const {return detConfidence_;};

        /**
         * NN's confidence about the detected class getter
         * @return confience of the estimated class of the detection
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

        BoundingBox2D bb_;
        float distance_;
        float detConfidence_;
        float classConfidence_;
        YoloDetectionClass cls_;
        std::vector<std::shared_ptr<DataModels::GenericDataModel>> parents_;

    };
}
