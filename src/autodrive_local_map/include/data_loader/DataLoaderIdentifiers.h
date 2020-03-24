#pragma once


namespace AutoDrive::DataLoader {

    enum class LidarIdentifier {
        kLeftLidar,
        kRightLidar,
        kNone,
    };

    enum class CameraIndentifier {
        kCameraLeftFront,
        kCameraLeftSide,
        kCameraRightFront,
        kCameraRightSide,
        kCameraIr,
        kErr
    };
}