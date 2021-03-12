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

#include <iostream>
#include <rtl/Core.h>
#include <rtl/Transformation.h>

namespace AtlasFusion::Algorithms {

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
