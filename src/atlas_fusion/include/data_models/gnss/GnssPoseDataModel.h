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

#include <cmath>
#include "data_models/GenericDataModel.h"

namespace AtlasFusion::DataModels {

    /**
     * GNSS Pose Data Model encapsulates global WGS84 position measured by GNSS receiver with differential antenna and
     * heading estimation
     */
    class GnssPoseDataModel : public GenericDataModel {

    public:

        /**
         * Constructor
         * @param timestamp measurement timestamp
         * @param lat WGS84 latitude
         * @param lon WGS84 longitude
         * @param alt WGS84 altitude
         * @param azim WGS84 azimuth
         */
        GnssPoseDataModel(uint64_t timestamp, double lat, double lon, double alt, double azim)
        : GenericDataModel(timestamp)
        , latitude_(lat)
        , longitude_(lon)
        , altitude_(alt)
        , azimut_(azim) {
            type_ = DataModelTypes::kGnssPositionDataModelType;
        }

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
         * WGS84 Altitude getter
         * @return altitude
         */
        double getAltitude() {return altitude_;};

        /**
         * Differential antenna estimated azimuth
         * @return azimuth
         */
        double getAzimut() {return azimut_;};

        /**
         * Differential antenna estimated heading = (-azimuth * pi / 180)
         * @return heading
         */
        double getHeading() {return -azimut_ * M_PI / 180;};

    private:

        double latitude_;
        double longitude_;
        double altitude_;
        double azimut_;
    };
}