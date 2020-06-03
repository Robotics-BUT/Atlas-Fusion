#pragma once

#include <iostream>
#include <rtl/Quaternion.h>
#include <rtl/Vector3D.h>
#include <rtl/Transformation3D.h>

namespace AutoDrive::Algorithms {

    /**
     * Imu Data Processor is responsible for a removing gravitation force from the meashured IMU data
     */
    class ImuDataProcessor {

    public:

        /**
         * Setter for a current IMU orientation
         * @param orient current imu orientation w.r.t. global coordinate system
         */
        void setOrientation(rtl::Quaternion<double> orient) { orientation_ = orient; };

        /**
         * Method removed gravitation forces from the given 3D acceleration measurement
         * @param acc rad acceleration data with gravitation component
         * @return gravitation-free linear acceleration data
         */
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
