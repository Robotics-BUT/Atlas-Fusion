#pragma once

#include <rtl/Vector3D.h>
#include <cmath>

#include "LocalPosition.h"

namespace AutoDrive::DataModels {

    class GlobalPosition {

    public:

        GlobalPosition(double lat, double lon, double alt, double azim)
        : position_{lat, lon, alt}
        , azim_(azim) {

        }

        double getLatitude() { return position_.x(); };
        double getLongitude() { return position_.y(); };
        double getAltitude() { return position_.z(); };
        double getAzimuth() { return azim_; };

        static inline double degToRad(double deg) {
            return deg * 3.1415 / 180;
        }

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



        static LocalPosition getOffsetBetweenCoords(GlobalPosition& coord1, GlobalPosition& coord2){

            LocalPosition offset;

            double mid_lat = (coord1.getLatitude() + coord2.getLatitude())/2;
            double mid_lon = (coord1.getLongitude() + coord2.getLongitude())/2;

            auto c1 = GlobalPosition(coord1.getLatitude(), mid_lon, 0, 0);
            auto c2 = GlobalPosition(coord2.getLatitude(), mid_lon, 0, 0);
            offset.setX(getDistanceBetweenCoords(c1, c2));

            c1 = GlobalPosition(mid_lat, coord1.getLongitude(), 0, 0);
            c2 = GlobalPosition(mid_lat, coord2.getLongitude(), 0, 0);
            offset.setY(getDistanceBetweenCoords(c1, c2));

            if (coord1.getLatitude() > coord2.getLatitude()) {
                offset.setX(-offset.getPosition().x());
            }
            if (coord1.getLongitude() < coord2.getLongitude()) {
                offset.setY(-offset.getPosition().y());
            }
            offset.setZ(coord2.getAltitude() - coord1.getAltitude());

            return offset;

        }

        static GlobalPosition localPoseToGlobalPose(LocalPosition localOffset, GlobalPosition globalOrigin) {

            const double step = 0.001f;
            auto g1 = GlobalPosition(globalOrigin.getLatitude() + step, globalOrigin.getLongitude() + step, 0, 0);
            LocalPosition mili_deg_distance = getOffsetBetweenCoords(globalOrigin, g1);

            double lat_step = (localOffset.getPosition().y() * step) / mili_deg_distance.getPosition().y();
            double lon_step = (localOffset.getPosition().x() * step) / mili_deg_distance.getPosition().x();

            GlobalPosition output{globalOrigin.getLatitude() + lat_step, globalOrigin.getLongitude() + lon_step, globalOrigin.getAltitude() + localOffset.getPosition().z(), 0};
            return output;
        }


    private:

        rtl::Vector3D<double> position_;
        double azim_;

    };

}