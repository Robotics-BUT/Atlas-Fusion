#include "visualizers/VisualizationHandler.h"

#include <sensor_msgs/PointCloud2.h>

#include "visualizers/Frames.h"

namespace AutoDrive::Visualizers{


    void VisualizationHandler::drawTestingCube() const{
        testCubePublisher_.publish(getTestCube());
    }


    void VisualizationHandler::drawLidarData(const std::shared_ptr<DataLoader::LidarScanDataModel> data) const {

        auto scan = data->getScan();
        if (data->getLidarIdentifier() == DataLoader::LidarIdentifier::kLeftLidar) {
            lidarVisualizer_.drawLeftPointcloud(scan);
        } else if (data->getLidarIdentifier() == DataLoader::LidarIdentifier::kRightLidar) {
            lidarVisualizer_.drawRightPointcloud(scan);
        }
    }


    void VisualizationHandler::drawImuData(const std::shared_ptr<DataLoader::ImuImuDataModel> data) const {
        imuVisualizer_.drawImuData(data);
    }


    void VisualizationHandler::drawRGBImage(const std::shared_ptr<DataLoader::CameraFrameDataModel> data) const {
        switch(data->getCameraIdentifier()) {
            case DataLoader::CameraIndentifier::kCameraLeftFront:
                cameraVisualizer_.drawCameraLeftFrontImage(data);
                break;
            case DataLoader::CameraIndentifier::kCameraLeftSide:
                cameraVisualizer_.drawCameraLeftSideImage(data);
                break;
            case DataLoader::CameraIndentifier::kCameraRightFront:
                cameraVisualizer_.drawCameraRightFrontImage(data);
                break;
            case DataLoader::CameraIndentifier::kCameraRightSide:
                cameraVisualizer_.drawCameraRightSideImage(data);
                break;
            default:
                logger_.warning("Unexpected camera frame source when drawing RGB image");
                break;
        }
    }

    void VisualizationHandler::drawIRImage(const std::shared_ptr<DataLoader::CameraIrFrameDataModel> data) const {
        cameraVisualizer_.drawCameraIrImage(data);
    }


    visualization_msgs::Marker VisualizationHandler::getTestCube() const {

        visualization_msgs::Marker cube;
        cube.header.frame_id = Frames::kOrigin;
        cube.header.stamp = ros::Time();
        cube.id = 0;
        cube.type = visualization_msgs::Marker::CUBE;
        cube.action = visualization_msgs::Marker::ADD;
        cube.pose.position.x = 0;
        cube.pose.position.y = 0;
        cube.pose.position.z = 0;
        cube.pose.orientation.x = 0.0;
        cube.pose.orientation.y = 0.0;
        cube.pose.orientation.z = 0.0;
        cube.pose.orientation.w = 1.0;
        cube.scale.x = 1.0;
        cube.scale.y = 1.0;
        cube.scale.z = 1.0;
        cube.color.a = 1.0;
        cube.color.r = 0.0;
        cube.color.g = 1.0;
        cube.color.b = 0.0;
        return cube;
    }


    void VisualizationHandler::drawGnssPoseData(const std::shared_ptr<DataLoader::GnssPoseDataModel>data) const {
        gnssVisualizer_.drawGnssPose(data);
    }

}