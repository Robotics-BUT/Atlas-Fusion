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

            //std::cout << " * * * * * * * * * " << std::endl;
            //std::cout << acc.x() << " "  << acc.y() <<  " " << acc.z() << std::endl;
            //std::cout << grav.x() << " "  << grav.y() <<  " " << grav.z() << std::endl;

            rtl::Vector3D<double> diff = acc - grav;

            //std::cout << diff.x() << " "  << diff.y() <<  " " << diff.z() << std::endl;
            return diff;
        }

    private:
        rtl::Quaternion<double> orientation_;
        rtl::Vector3D<double> g_ = {0.0, 0.0, 9.81};
    };

}
