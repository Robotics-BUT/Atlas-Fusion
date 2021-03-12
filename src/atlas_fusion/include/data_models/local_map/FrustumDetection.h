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
#include <memory>
#include <rtl/Core.h>

#include "data_models/yolo/YoloDetection.h"

namespace AtlasFusion::DataModels {


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
        explicit FrustumDetection(std::shared_ptr<const rtl::Frustum3D<double>> frustum, float detConfidence, YoloDetectionClass cls)
                : frustum_{std::move(frustum)}
                , detConfidence_{detConfidence}
                , cls_{cls} {

        }

        /**
         * Getter for frustum.
         * @return 3D space structure
         */
        std::shared_ptr<const rtl::Frustum3D<double>> getFrustum() const { return frustum_; };

        /**
         * NN's confidence of the detection getter
         * @return confidence of the detection
         */
        float getDetectionConfidence() const {return detConfidence_;};

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

        std::shared_ptr<const rtl::Frustum3D<double>> frustum_;
        float detConfidence_;
        YoloDetectionClass cls_;
        std::vector<std::shared_ptr<DataModels::GenericDataModel>> parents_;

    };
}
