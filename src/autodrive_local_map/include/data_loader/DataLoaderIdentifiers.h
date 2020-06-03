#pragma once


namespace AutoDrive::DataLoader {

    /**
     * Lidar Data Loaders identifiers
     */
    enum class LidarIdentifier {
        kLeftLidar,
        kRightLidar,
        kNone,
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
        kErr
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