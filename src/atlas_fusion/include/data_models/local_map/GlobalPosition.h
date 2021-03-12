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

#include <rtl/Core.h>
#include <cmath>

#include "LocalPosition.h"

namespace AtlasFusion::DataModels {

    /**
     * Global Position represents WGS84 global position
     */
    class GlobalPosition {

    public:

        /**
         * Constructor
         * @param lat WGS84 latitude
         * @param lon WGS84 longitude
         * @param alt WGS84 altitude
         * @param azim azimut
         */
        GlobalPosition(double lat, double lon, double alt, double azim)
        : position_{lat, lon, alt}
        , azim_(azim) {

        }

        /**
         * Latitude getter
         * @return latitude
         */
        double getLatitude() { return position_.x(); };

        /**
         * Logitude getter
         * @return longitude
         */
        double getLongitude() { return position_.y(); };

        /**
         * Altitude getter
         * @return altitude
         */
        double getAltitude() { return position_.z(); };

        /**
         * Azimut getter
         * @return azimut
         */
        double getAzimuth() { return azim_; };

        /**
         * Converts degrees to radians
         * @param deg angle in degrees
         * @return angle in radians
         */
        static inline double degToRad(double deg) {
            return deg * 3.1415 / 180;
        }

        /**
         * Method calculates metric distance between two WGS84 coordinates
         * @param coord1 first WGS84 coordinate
         * @param coord2 second WGS84 coordinate
         * @return metrid distance between coords
         */
        static double getDistanceBetweenCoords(GlobalPosition& coord1, GlobalPosition& coord2){

            const double R = 6371000;

            double lat_1 = degToRad(coord1.getLatitude());
            double lat_2 = degToRad(coord2.getLatitude());

            double lon_1 = degToRad(coord1.getLongitude());
            double lon_2 = degToRad(coord2.getLongitude());

            double delta_lat = lat_2 - lat_1;
            double delta_lon = lon_2 - lon_1;

            double a = std::sin(delta_lat/2) * std::sin(delta_lat/2) + std::cos(lat_1) * std::cos(lat_2) * std::sin(delta_lon/2) * std::sin(delta_lon/2);
            double c = 2* std::atan2(std::sqrt(a), std::sqrt(1-a));
            double d = R * c;
            return d;
        }


        /**
         * Returns the 2D vector of metrix distances between two coordinates. Positive direction to north and east.
         * @param coord1 Reference coord
         * @param coord2 Vector coord
         * @return (x, y, 0) distance between two WGS84 coordinates
         */
        static rtl::Vector3D<double> getOffsetBetweenCoords(GlobalPosition& coord1, GlobalPosition& coord2){

            rtl::Vector3D<double> offset{0,0,0};

            double mid_lat = (coord1.getLatitude() + coord2.getLatitude())/2;
            double mid_lon = (coord1.getLongitude() + coord2.getLongitude())/2;

            auto c1 = GlobalPosition(coord1.getLatitude(), mid_lon, 0, 0);
            auto c2 = GlobalPosition(coord2.getLatitude(), mid_lon, 0, 0);
            offset.setX(getDistanceBetweenCoords(c1, c2));

            c1 = GlobalPosition(mid_lat, coord1.getLongitude(), 0, 0);
            c2 = GlobalPosition(mid_lat, coord2.getLongitude(), 0, 0);
            offset.setY(getDistanceBetweenCoords(c1, c2));

            if (coord1.getLatitude() > coord2.getLatitude()) {
                offset.setX(-offset.x());
            }
            if (coord1.getLongitude() < coord2.getLongitude()) {
                offset.setY(-offset.y());
            }
            offset.setZ(coord2.getAltitude() - coord1.getAltitude());

            return offset;

        }

        /**
         * Converst relative local metric position referenced to WGS84 global anchor into the nwe WGS84 position.
         * @param localOffset local position that will be converted
         * @param globalOrigin WGS position of the local origin
         * @return WGS84 position of the input local offset
         */
        static GlobalPosition localPoseToGlobalPose(LocalPosition localOffset, GlobalPosition globalOrigin) {

            const double step = 0.001f;
            auto g1 = GlobalPosition(globalOrigin.getLatitude() + step, globalOrigin.getLongitude() + step, 0, 0);
            auto mili_deg_distance = getOffsetBetweenCoords(globalOrigin, g1);

            double lat_step = (localOffset.getPosition().y() * step) / mili_deg_distance.y();
            double lon_step = (localOffset.getPosition().x() * step) / mili_deg_distance.x();

            GlobalPosition output{globalOrigin.getLatitude() + lat_step, globalOrigin.getLongitude() + lon_step, globalOrigin.getAltitude() + localOffset.getPosition().z(), 0};
            return output;
        }


    private:

        rtl::Vector3D<double> position_;
        double azim_;

    };

}