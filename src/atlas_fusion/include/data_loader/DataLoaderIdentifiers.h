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


namespace AtlasFusion::DataLoader {

    /**
     * Lidar Data Loaders identifiers
     */
    enum class LidarIdentifier {
        kLeftLidar,
        kRightLidar,
        kCenterLidar,
        kNone,
    };

    enum class RadarIdentifier {
        kRadarTi,
    };

    /**
     * Camera Data Loader identifiers
     */
    enum class CameraIndentifier {
        kCameraLeftFront,
        kCameraLeftSide,
        kCameraRightFront,
        kCameraRightSide,
        kCameraIr,
        kErr,
    };

    /**
     * Imu Loader Identifier distinguishes Imu Data Loaders by the data types they are handling
     */
    enum class ImuLoaderIdentifier {
        kDQuat,
        kGnss,
        kImu,
        kMag,
        kPressure,
        kTemp,
        kTime,
    };

    /**
     * GNSS Loader Identifier distinguishes GNSS Data Loaders by the data types they are handling
     */
    enum class GnssLoaderIdentifier {
        kPose,
        kTime,
    };
}