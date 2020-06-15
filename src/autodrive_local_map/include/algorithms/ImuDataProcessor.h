#pragma once

#include <iostream>
#include <rtl/Core.h>
#include <rtl/Transformation.h>

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

            auto tf = rtl::RigidTf3D<double>{ orientation_, {0,0,0}};
            auto grav = tf.inverted()(g_);
            rtl::Vector3D<double> diff = acc - grav;
            return diff;
        }

    private:
        rtl::Quaternion<double> orientation_ = rtl::Quaternion<double>::identity();
        rtl::Vector3D<double> g_ = {0.0, 0.0, 9.81};
    };

}
