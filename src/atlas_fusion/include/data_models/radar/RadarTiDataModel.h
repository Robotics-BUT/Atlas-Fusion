/*
 * Copyright 2021 Brno University of Technology
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

#include "data_models/GenericDataModel.h"
#include <rtl/Core.h>

namespace AtlasFusion::DataModels {

    /**
     * Single detection done by mm-wave TI Radar
     */
    class RadarTiDataModel : public GenericDataModel {

    public:

        class Object {

        public:

            Object(rtl::Vector3f pose, float velocity)
                    : pose_{pose}
                    , velocity_{velocity} {}

            /**
             * Get position of detected object
             * @return postion
             */
            inline const rtl::Vector3f& getPose() const { return pose_; };

            /**
             * Get position of detected object
             * @return postion
             */
            inline float getVelocity() const { return velocity_; };

        private:
            rtl::Vector3f pose_;
            float velocity_;
        };


        /**
         * Constructor
         * @param timestamp recording session timestamp
         * @param vector of scanned objects
         */
        RadarTiDataModel(const uint64_t timestamp, const std::vector<Object>& objects)
                : GenericDataModel(timestamp)
                , objects_{objects} {
            type_ = DataModelTypes::kRadarTiScanDataModelType;
        }

        /**
         * Constructor
         * @param timestamp recording session timestamp
         */
        RadarTiDataModel(const uint64_t timestamp)
                : GenericDataModel(timestamp)
                , objects_{} {
            type_ = DataModelTypes::kRadarTiScanDataModelType;
        }

        std::string toString() override;

        /**
         * Get vector of all objects detected by the scan
         * @return postion
         */
        inline const std::vector<Object>& getObjects() const { return objects_; };

    private:
        std::vector<Object> objects_;
    };
}