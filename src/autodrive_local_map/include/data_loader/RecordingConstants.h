#pragma once

#include <iostream>
#include <vector>

namespace AutoDrive {
    namespace DataLoader {
        namespace Folders {

            const std::string kCameraLeftFrontFolder = "camera_left_front/";
            const std::string kCameraLeftSideFolder = "camera_left_side/";
            const std::string kCameraRightFrontFolder = "camera_right_front/";
            const std::string kCameraRightSideFolder = "camera_right_side/";

            const std::string kCameraIr = "camera_ir/";

            const std::string kLidarLeftFolder = "lidar_left/";
            const std::string kLidarRightFolder = "lidar_right/";

            const std::string kImuFolder = "imu/";
            const std::string kGnssFolder = "gnss/";

            const std::string kYoloFolder = "yolo/";

            const std::vector <std::string> mandatoryFolders{
                    kCameraLeftFrontFolder,
                    kCameraLeftSideFolder,
                    kCameraRightFrontFolder,
                    kCameraRightSideFolder,
                    kCameraIr,
                    kLidarLeftFolder,
                    kLidarRightFolder,
                    kImuFolder,
                    kGnssFolder
            };
        }

        namespace Files {

            const std::string kVideoFile = "video.mp4";
            const std::string kTimestampFile = "timestamps.txt";

            const std::string kDquatFile = "d_quat.txt";
            const std::string kGnssFile = "gnss.txt";
            const std::string kImuFile = "imu.txt";
            const std::string kMagFile = "mag.txt";
            const std::string kPressureFile = "pressure.txt";
            const std::string kTimeFile = "time.txt";
            const std::string kTempFile = "temp.txt";

            const std::string kPoseFile = "pose.txt";

            const std::string kScanFile = "scan";
            const std::string kPcdExt = ".pcd";

            const std::string kCameraLeftFrontCalibYaml = "camera_left_front.yaml";
            const std::string kCameraLeftSideCalibYaml = "camera_left_side.yaml";
            const std::string kCameraRightFrontCalibYaml = "camera_right_front.yaml";
            const std::string kCameraRightSideCalibYaml = "camera_right_side.yaml";
            const std::string kCameraIrCalibYaml = "camera_ir.yaml";

            const std::vector <std::string> mandatoryFiles {
                    Folders::kCameraIr + kVideoFile,
                    Folders::kCameraIr + kTimestampFile,

                    Folders::kCameraLeftFrontFolder + kVideoFile,
                    Folders::kCameraLeftFrontFolder + kTimestampFile,

                    Folders::kCameraLeftSideFolder + kVideoFile,
                    Folders::kCameraLeftSideFolder + kTimestampFile,

                    Folders::kCameraRightFrontFolder + kVideoFile,
                    Folders::kCameraRightFrontFolder + kTimestampFile,

                    Folders::kCameraRightSideFolder + kVideoFile,
                    Folders::kCameraRightSideFolder + kTimestampFile,

                    Folders::kLidarLeftFolder + kTimestampFile,
                    Folders::kLidarRightFolder + kTimestampFile,

                    Folders::kGnssFolder + kPoseFile,
                    Folders::kGnssFolder + kTimeFile,

                    Folders::kImuFolder + kDquatFile,
                    Folders::kImuFolder + kImuFile,
                    Folders::kImuFolder + kGnssFile,
                    Folders::kImuFolder + kMagFile,
                    Folders::kImuFolder + kPressureFile,
                    Folders::kImuFolder + kTimeFile,
                    Folders::kImuFolder + kTempFile,
            };
        }
    }
}
