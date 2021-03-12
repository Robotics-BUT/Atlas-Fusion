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

#include "data_models/GenericDataModel.h"

namespace AtlasFusion::DataModels {

    /**
     * XSens IMU contains also a GNSS receiver. IMU GNSS Data Model represents global WGS84 position
     */
    class ImuGnssDataModel : public GenericDataModel {

    public:

        /**
         * Constructor
         * @param timestamp measurement timestamp
         * @param lat WGS84 latitude
         * @param lon WGS84 longitude
         * @param alt WGS84 altitude
         */
        ImuGnssDataModel(uint64_t timestamp, double lat, double lon, double alt)
        : GenericDataModel(timestamp)
        , latitude_(lat)
        , longitude_(lon)
        , altitude_(alt) {
            type_ = DataModelTypes::kImuGnssDataModelType;
        };

        std::string toString() override;

        /**
         * WGS84 latitude getter
         * @return latitude
         */
        double getLatitude() {return latitude_;};

        /**
         * WGS84 longitude getter
         * @return longitude
         */
        double getLongitude() {return longitude_;};

        /**
         * WGS84 altitude getter
         * @return altitude
         */
        double getAltitude() {return altitude_;};

    private:

        double latitude_;
        double longitude_;
        double altitude_;

    };
}

