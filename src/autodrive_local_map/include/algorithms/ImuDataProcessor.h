#pragma once

#include <iostream>
#include <rtl/Quaternion.h>
#include <rtl/Vector3D.h>
#include <rtl/Transformation3D.h>

namespace AutoDrive::Algorithms {

    class ImuDataProcessor {

    public:

        void setOrientation(rtl::Quaternion<double> orient) { orientation_ = orient; };

        rtl::Vector3D<double> removeGravitaionAcceleration(rtl::Vector3D<double> acc) {

            auto tf = rtl::Transformation3D<double>{ orientation_, {0,0,0}};
            auto grav = tf.inverted()(g_);
            rtl::Vector3D<double> diff = acc - grav;
            return diff;
        }

    private:
        rtl::Quaternion<double> orientation_;
        rtl::Vector3D<double> g_ = {0.0, 0.0, 9.81};
    };

}
