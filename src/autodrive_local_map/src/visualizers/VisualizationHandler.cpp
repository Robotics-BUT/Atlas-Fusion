#include "visualizers/VisualizationHandler.h"

#include <sensor_msgs/PointCloud2.h>

#include "local_map/Frames.h"
#include "visualizers/VisualizationStructures.h"

namespace AutoDrive::Visualizers{


    void VisualizationHandler::drawTestingCube() const{
        testCubePublisher_.publish(getTestCube());
    }

    void VisualizationHandler::drawLidarData(const std::shared_ptr<DataModels::LidarScanDataModel> data) const {

        auto scan = data->getScan();
        if (data->getLidarIdentifier() == DataLoader::LidarIdentifier::kLeftLidar) {
            lidarVisualizer_.drawLeftPointcloud(scan);
        } else if (data->getLidarIdentifier() == DataLoader::LidarIdentifier::kRightLidar) {
            lidarVisualizer_.drawRightPointcloud(scan);
        }
    }

    void VisualizationHandler::drawImuData(const rtl::Vector3D<double> linAcc) const {
        imuVisualizer_.drawImuData(linAcc);
    }

    void VisualizationHandler::drawRGBImage(const std::shared_ptr<DataModels::CameraFrameDataModel> data) const {
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
                context_.logger_.warning("Unexpected camera frame source when drawing RGB image");
                break;
        }
    }

    void VisualizationHandler::drawIRImage(const std::shared_ptr<DataModels::CameraIrFrameDataModel> data) const {
        cameraVisualizer_.drawCameraIrImage(data);
    }

    visualization_msgs::Marker VisualizationHandler::getTestCube() const {

        visualization_msgs::Marker cube;
        cube.header.frame_id = LocalMap::Frames::kOrigin;
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

    void VisualizationHandler::drawGnssPoseData(const std::shared_ptr<DataModels::GnssPoseDataModel>data) const {
        gnssVisualizer_.drawGnssPose(data);
    }

    void VisualizationHandler::drawRawGnssTrajectory(const std::deque<DataModels::LocalPosition> &data) const {
        trajectoryVisualizer_.drawRawTrajectory(data);
    }

    void VisualizationHandler::drawFilteredTrajectory(const std::deque<DataModels::LocalPosition> &data) const {
        trajectoryVisualizer_.drawFilteredTrajectory(data);
    }

    void VisualizationHandler::drawImuGpsTrajectory(const std::deque<DataModels::LocalPosition> &data) const {
        trajectoryVisualizer_.drawImuGpsTrajectory(data);
    }

    void VisualizationHandler::updateOriginToRootTf(const DataModels::LocalPosition& pose) {
        rtl::Transformation3D tf(pose.getOrientation(), pose.getPosition());
        tfTreeVisualizer_.updateOriginToRootTf(tf);
    }

    void VisualizationHandler::setCameraCalibParamsForCameraId(std::shared_ptr<DataModels::CameraCalibrationParamsDataModel> params, DataLoader::CameraIndentifier id) {
        cameraParams_[id] = params;
        cameraVisualizer_.setCameraParams(cameraParams_);
    }

    void VisualizationHandler::drawFrustumDetections(std::vector<std::shared_ptr<DataModels::FrustumDetection>> detections) {
        frustumVisualizer_.visualizeFrustumDetections(detections);
    }

    void VisualizationHandler::drawAggregatedPointcloud(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pc) {
        lidarVisualizer_.drawPointcloudOnTopic(pc, "test_topic");
    }


    void VisualizationHandler::drawAggregatedLasers(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pc) {
        lidarVisualizer_.drawLasers(pc);
    }
}