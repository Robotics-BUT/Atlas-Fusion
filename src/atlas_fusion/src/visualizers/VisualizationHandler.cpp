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

#include "visualizers/VisualizationHandler.h"

#include <sensor_msgs/PointCloud2.h>
#include "Timer.h"

namespace AutoDrive::Visualizers {

    void VisualizationHandler::drawSelfGlobal() const {
        selfGlobalPublisher_.publish(getSelfGlobalCube());
    }

    void VisualizationHandler::drawSelfEgo() const {
        selfEgoPublisher_.publish(getSelfEgoCube());
    }

    /* LIDAR */

    void VisualizationHandler::drawLidarData(const std::shared_ptr<DataModels::LidarScanDataModel> &data) {

        if (!context_.getFunctionalityFlags().visualization_global_enable_) { return; }
        if (!context_.getFunctionalityFlags().lidar_visualization_) { return; }

        auto scan = data->getScan();
        if (data->getLidarIdentifier() == DataLoader::LidarIdentifier::kLeftLidar) {
            lidarVisualizer_.drawPointCloudOnTopic(scan, Topics::kLidarLeft, FrameType::kLidarLeft);
        } else if (data->getLidarIdentifier() == DataLoader::LidarIdentifier::kRightLidar) {
            lidarVisualizer_.drawPointCloudOnTopic(scan, Topics::kLidarRight, FrameType::kLidarRight);
        } else if (data->getLidarIdentifier() == DataLoader::LidarIdentifier::kCenterLidar) {
            lidarVisualizer_.drawPointCloudOnTopic(scan, Topics::kLidarCenter, FrameType::kLidarCenter);
        }
    }


    void VisualizationHandler::drawAggregatedPointCloudGlobal(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &pc) {
        if (!context_.getFunctionalityFlags().visualization_global_enable_) { return; }
        if (!context_.getFunctionalityFlags().lidar_visualization_) { return; }

        lidarVisualizer_.drawPointCloudOnTopic(pc, Topics::kLidarAggregatedGlobal, FrameType::kOrigin);
    }

    void VisualizationHandler::drawAggregatedPointCloudEgo(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &pc) {
        if (!context_.getFunctionalityFlags().visualization_global_enable_) { return; }
        if (!context_.getFunctionalityFlags().lidar_visualization_) { return; }

        lidarVisualizer_.drawPointCloudOnTopic(pc, Topics::kLidarAggregatedEgo, FrameType::kOrigin);
    }


    void VisualizationHandler::drawAggregatedLasers(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &pc) {

        if (!context_.getFunctionalityFlags().visualization_global_enable_) { return; }
        if (!context_.getFunctionalityFlags().lidar_visualization_) { return; }

        lidarVisualizer_.drawPointCloudOnTopic(pc, Topics::kLidarLaser, FrameType::kOrigin);
    }

    void VisualizationHandler::drawLidarApproximations(std::shared_ptr<std::vector<rtl::LineSegment3D<double>>> ls) {

        if (!context_.getFunctionalityFlags().visualization_global_enable_) { return; }
        if (!context_.getFunctionalityFlags().lidar_visualization_) { return; }

        visualization_msgs::Marker::_color_type col;
        col.a = 1;
        col.r = 0;
        col.g = 1;
        col.b = 0;
        lidarVisualizer_.drawApproximationOnTopic(ls, Topics::kLidarApproximation, FrameType::kOrigin, col);
    }

    void VisualizationHandler::drawLidarApproximationsRoad(std::shared_ptr<std::vector<rtl::LineSegment3D<double>>> ls) {

        if (!context_.getFunctionalityFlags().visualization_global_enable_) { return; }
        if (!context_.getFunctionalityFlags().lidar_visualization_) { return; }

        visualization_msgs::Marker::_color_type col;
        col.a = 1;
        col.r = 1;
        col.g = 0;
        col.b = 0;
        lidarVisualizer_.drawApproximationOnTopic(ls, Topics::kLidarApproximationRoad, FrameType::kOrigin, col);
    }

    void VisualizationHandler::drawGlobalPointcloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &pc) {
        if (!context_.getFunctionalityFlags().visualization_global_enable_) { return; }
        if (!context_.getFunctionalityFlags().lidar_visualization_) { return; }

        lidarVisualizer_.drawPointCloudOnTopic(pc, Topics::kGlobalPointCloud, FrameType::kOrigin);
    }

    void VisualizationHandler::drawPointcloudCutout(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &pc) {

        if (!context_.getFunctionalityFlags().visualization_global_enable_) { return; }
        if (!context_.getFunctionalityFlags().lidar_visualization_) { return; }

        lidarVisualizer_.drawPointCloudOnTopic(pc, Topics::kCutoutPointcloud, FrameType::kImu);
    }


    void VisualizationHandler::drawRGBImage(const std::shared_ptr<DataModels::CameraFrameDataModel> &data) {

        if (!context_.getFunctionalityFlags().visualization_global_enable_) { return; }
        if (!context_.getFunctionalityFlags().rgb_camera_visualization_) { return; }

        switch (data->getCameraIdentifier()) {
            case DataLoader::CameraIndentifier::kCameraLeftFront:
                cameraVisualizer_.drawRGBCameraFrameWithTopic(data, Topics::kCameraLeftFront, Topics::kCameraLeftFrontInfo, FrameType::kCameraLeftFront);
                break;
            case DataLoader::CameraIndentifier::kCameraLeftSide:
                cameraVisualizer_.drawRGBCameraFrameWithTopic(data, Topics::kCameraLeftSide, Topics::kCameraLeftSideInfo, FrameType::kCameraLeftSide);
                break;
            case DataLoader::CameraIndentifier::kCameraRightFront:
                cameraVisualizer_.drawRGBCameraFrameWithTopic(data, Topics::kCameraRightFront, Topics::kCameraRightFrontInfo, FrameType::kCameraRightFront);
                break;
            case DataLoader::CameraIndentifier::kCameraRightSide:
                cameraVisualizer_.drawRGBCameraFrameWithTopic(data, Topics::kCameraRightSide, Topics::kCameraRightSideInfo, FrameType::kCameraRightSide);
                break;
            default:
                context_.logger_.warning("Unexpected camera frame source when drawing RGB image");
                break;
        }
    }


    void VisualizationHandler::drawIRImage(const std::shared_ptr<DataModels::CameraIrFrameDataModel> &data) {

        if (!context_.getFunctionalityFlags().visualization_global_enable_) { return; }
        if (!context_.getFunctionalityFlags().ir_camera_visualization_) { return; }

        cameraVisualizer_.drawIRCameraFrameWithTopic(data, Topics::kCameraIr, Topics::kCameraIrInfo, FrameType::kCameraIr);
    }


    void VisualizationHandler::drawVelocityData(const rtl::Vector3D<double> &speed) {

        if (!context_.getFunctionalityFlags().visualization_global_enable_) { return; }

        imuVisualizer_.drawImuData(speed, FrameType::kImu, Topics::kSpeedTopic);
    }

    void VisualizationHandler::drawImuData(const rtl::Vector3D<double> &linAcc) {

        if (!context_.getFunctionalityFlags().visualization_global_enable_) { return; }
        if (!context_.getFunctionalityFlags().imu_visualization_) { return; }

        imuVisualizer_.drawImuData(linAcc, FrameType::kImu, Topics::kImuTopic);
    }

    void VisualizationHandler::drawImuAvgData(const rtl::Vector3D<double> &linAcc) {

        if (!context_.getFunctionalityFlags().visualization_global_enable_) { return; }
        if (!context_.getFunctionalityFlags().imu_visualization_) { return; }

        imuVisualizer_.drawImuData(linAcc, FrameType::kImu, Topics::kImuAvgTopic);
    }

    void VisualizationHandler::drawGnssPoseData(const std::shared_ptr<DataModels::GnssPoseDataModel> &data) const {

        if (!context_.getFunctionalityFlags().visualization_global_enable_) { return; }
        if (!context_.getFunctionalityFlags().gnss_visualization_) { return; }

        gnssVisualizer_.drawGnssPose(data);
    }

    void VisualizationHandler::drawRawGnssTrajectory(const std::deque<DataModels::LocalPosition> &data) const {

        if (!context_.getFunctionalityFlags().visualization_global_enable_) { return; }
        if (!context_.getFunctionalityFlags().gnss_visualization_) { return; }

        trajectoryVisualizer_.drawRawTrajectory(data);
    }

    void VisualizationHandler::drawFilteredTrajectory(const std::deque<DataModels::LocalPosition> &data) const {

        if (!context_.getFunctionalityFlags().visualization_global_enable_) { return; }
        if (!context_.getFunctionalityFlags().gnss_visualization_) { return; }

        trajectoryVisualizer_.drawFilteredTrajectory(data);
    }

    void VisualizationHandler::drawImuGpsTrajectory(const std::deque<DataModels::LocalPosition> &data) const {

        if (!context_.getFunctionalityFlags().visualization_global_enable_) { return; }
        if (!context_.getFunctionalityFlags().imu_visualization_) { return; }

        trajectoryVisualizer_.drawImuGpsTrajectory(data);
    }

    void VisualizationHandler::updateOriginToRootTf(const DataModels::LocalPosition &pose) {

        if (!context_.getFunctionalityFlags().visualization_global_enable_) { return; }

        rtl::RigidTf3D<double> tf(pose.getOrientation(), pose.getPosition());
        tfTreeVisualizer_.updateOriginToRootTf(tf);
    }

    void VisualizationHandler::setCameraCalibParamsForCameraId(DataModels::CameraCalibrationParamsDataModel &params, const FrameType &frame) {
        cameraVisualizer_.setCameraParams(frame, params);
    }

    void VisualizationHandler::drawFrustumDetections(const std::vector<DataModels::FrustumDetection> &detections) {
        if (!context_.getFunctionalityFlags().visualization_global_enable_) { return; }
        frustumVisualizer_.visualizeFrustumDetections(detections);
    }

    void VisualizationHandler::drawLidarDetection(const std::vector<std::shared_ptr<DataModels::LidarDetection>> &detections) {
        if (!context_.getFunctionalityFlags().visualization_global_enable_) { return; }
        lidarVisualizer_.drawLidarDetections(detections, Topics::kLidarDetections, FrameType::kImu);
    }

    void VisualizationHandler::drawTelemetry(const std::string &telemetryText) {
        if (!context_.getFunctionalityFlags().visualization_global_enable_) { return; }
        telemetryVisualizer_.drawTelemetryAsText(telemetryText, FrameType::kImu, Topics::kTelemetryText);
    }

    void VisualizationHandler::drawRadarTiObjects(const std::vector<DataModels::RadarTiDataModel::Object> &objects) {

        if (!context_.getFunctionalityFlags().visualization_global_enable_) { return; }
        if (!context_.getFunctionalityFlags().radar_visualization_) { return; }

        radarVisualizer_.drawRadarDetectionsOnTopic(objects, Topics::kRadarTiObjects, FrameType::kRadarTi);
    }

    void VisualizationHandler::drawSensorStatus(const std::string &sensorStatus, const FrameType &frameType) {
        if (!context_.getFunctionalityFlags().visualization_global_enable_) { return; }

        std::string topic;
        switch(frameType) {
            case FrameType::kOrigin:
            case FrameType::kLidarLeft:
            case FrameType::kLidarRight:
            case FrameType::kLidarCenter:
            case FrameType::kRadarTi:
            case FrameType::kGnssAntennaFront:
            case FrameType::kGnssAntennaRear:
            case FrameType::kImu: return;
            case FrameType::kCameraLeftFront:
                topic = Topics::kCameraLeftFrontStatus; break;
            case FrameType::kCameraLeftSide:
                topic = Topics::kCameraLeftSideStatus; break;
            case FrameType::kCameraRightFront:
                topic = Topics::kCameraRightFrontStatus; break;
            case FrameType::kCameraRightSide:
                topic = Topics::kCameraRightSideStatus; break;
            case FrameType::kCameraIr:
                topic = Topics::kCameraIrStatus; break;

        }
        sensorStatusVisualizer_.drawStatusAsText(sensorStatus, frameType, topic);
    }

    visualization_msgs::Marker VisualizationHandler::getSelfEgoCube() const {
        visualization_msgs::Marker cube;
        cube.header.frame_id = frameTypeName(FrameType::kOrigin);
        cube.header.stamp = ros::Time();
        cube.id = 0;
        cube.type = visualization_msgs::Marker::CUBE;
        cube.action = visualization_msgs::Marker::ADD;
        cube.pose.position.x = 0;
        cube.pose.position.y = 0;
        cube.pose.position.z = -0.75;
        cube.pose.orientation.x = 0.0;
        cube.pose.orientation.y = 0.0;
        cube.pose.orientation.z = 0.0;
        cube.pose.orientation.w = 1.0;
        cube.scale.x = 4.5;
        cube.scale.y = 2.5;
        cube.scale.z = 1.5;

        cube.color.a = 0.5;
        cube.color.r = 0.0;
        cube.color.g = 1.0;
        cube.color.b = 0.0;
        return cube;
    }

    visualization_msgs::Marker VisualizationHandler::getSelfGlobalCube() const {
        visualization_msgs::Marker cube;
        cube.header.frame_id = frameTypeName(FrameType::kImu);
        cube.header.stamp = ros::Time();
        cube.id = 0;
        cube.type = visualization_msgs::Marker::CUBE;
        cube.action = visualization_msgs::Marker::ADD;
        cube.pose.position.x = 0;
        cube.pose.position.y = 0;
        cube.pose.position.z = -0.75;
        cube.pose.orientation.x = 0.0;
        cube.pose.orientation.y = 0.0;
        cube.pose.orientation.z = 0.0;
        cube.pose.orientation.w = 1.0;
        cube.scale.x = 4.5;
        cube.scale.y = 2.5;
        cube.scale.z = 1.5;

        cube.color.a = 0.5;
        cube.color.r = 0.0;
        cube.color.g = 1.0;
        cube.color.b = 0.0;
        return cube;
    }
}