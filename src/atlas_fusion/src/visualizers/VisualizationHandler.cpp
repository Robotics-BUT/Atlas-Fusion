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

#include "local_map/Frames.h"
#include "visualizers/VisualizationStructures.h"

namespace AtlasFusion::Visualizers{


    void VisualizationHandler::drawTestingCube() const{
        testCubePublisher_.publish(getTestCube());
    }


    void VisualizationHandler::drawSelf() const {
        selfPublisher_.publish(getSelfCube());
    }

    /* LIDAR */

    void VisualizationHandler::drawLidarData(const std::shared_ptr<DataModels::LidarScanDataModel> data) {

        if (!context_.getFunctionalityFlags().visualization_global_enable_) {return;}
        if (!context_.getFunctionalityFlags().lidar_visualization_) {return;}

        auto scan = data->getScan();
        if (data->getLidarIdentifier() == DataLoader::LidarIdentifier::kLeftLidar) {
            lidarVisualizer_.drawPointcloudOnTopic(scan, Topics::kLidarLeft, LocalMap::Frames::kLidarLeft);
        } else if (data->getLidarIdentifier() == DataLoader::LidarIdentifier::kRightLidar) {
            lidarVisualizer_.drawPointcloudOnTopic(scan, Topics::kLidarRight, LocalMap::Frames::kLidarRight);
        }else if (data->getLidarIdentifier() == DataLoader::LidarIdentifier::kCenterLidar) {
            lidarVisualizer_.drawPointcloudOnTopic(scan, Topics::kLidarCenter, LocalMap::Frames::kLidarCenter);
        }
    }


    void VisualizationHandler::drawAggregatedPointcloud(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pc) {

        if (!context_.getFunctionalityFlags().visualization_global_enable_) {return;}
        if (!context_.getFunctionalityFlags().lidar_visualization_) {return;}

        lidarVisualizer_.drawPointcloudOnTopic(pc, Topics::kLidarAggregated, LocalMap::Frames::kOrigin);
    }


    void VisualizationHandler::drawAggregatedLasers(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pc) {

        if (!context_.getFunctionalityFlags().visualization_global_enable_) {return;}
        if (!context_.getFunctionalityFlags().lidar_visualization_) {return;}

        lidarVisualizer_.drawPointcloudOnTopic(pc, Topics::kLidarLaser, LocalMap::Frames::kOrigin);
    }

    void VisualizationHandler::drawLidarApproximations(std::shared_ptr<std::vector<rtl::LineSegment3D<double>>> ls) {

        if (!context_.getFunctionalityFlags().visualization_global_enable_) {return;}
        if (!context_.getFunctionalityFlags().lidar_visualization_) {return;}

        visualization_msgs::Marker::_color_type col;
        col.a = 1;
        col.r = 0;
        col.g = 1;
        col.b = 0;
        lidarVisualizer_.drawApproximationOnTopic(ls, Topics::kLidarApproximation, LocalMap::Frames::kOrigin, col);
    }

    void VisualizationHandler::drawLidarApproximationsRoad(std::shared_ptr<std::vector<rtl::LineSegment3D<double>>> ls) {

        if (!context_.getFunctionalityFlags().visualization_global_enable_) {return;}
        if (!context_.getFunctionalityFlags().lidar_visualization_) {return;}

        visualization_msgs::Marker::_color_type col;
        col.a = 1;
        col.r = 1;
        col.g = 0;
        col.b = 0;
        lidarVisualizer_.drawApproximationOnTopic(ls, Topics::kLidarApproximationRoad, LocalMap::Frames::kOrigin, col);
    }

    void VisualizationHandler::drawGlobalPointcloud(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pc) {
        if (!context_.getFunctionalityFlags().visualization_global_enable_) {return;}
        if (!context_.getFunctionalityFlags().lidar_visualization_) {return;}

        lidarVisualizer_.drawPointcloudOnTopic(pc, Topics::kGlobalPointCloud, LocalMap::Frames::kOrigin);
    }

    void VisualizationHandler::drawPointcloudCutout(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pc) {

        if (!context_.getFunctionalityFlags().visualization_global_enable_) {return;}
        if (!context_.getFunctionalityFlags().lidar_visualization_) {return;}

        lidarVisualizer_.drawPointcloudOnTopic(pc, Topics::kCutoutPointcloud, LocalMap::Frames::kImuFrame);
    }


    void VisualizationHandler::drawRGBImage(const std::shared_ptr<DataModels::CameraFrameDataModel> data) {

        if (!context_.getFunctionalityFlags().visualization_global_enable_) {return;}
        if (!context_.getFunctionalityFlags().rgb_camera_visualization_) {return;}

        switch(data->getCameraIdentifier()) {
            case DataLoader::CameraIndentifier::kCameraLeftFront:
                cameraVisualizer_.drawRGBCameraFrameWithTopic(data, Topics::kCameraLeftFront, Topics::kCameraLeftFrontInfo, LocalMap::Frames::kCameraLeftFront);
                break;
            case DataLoader::CameraIndentifier::kCameraLeftSide:
                cameraVisualizer_.drawRGBCameraFrameWithTopic(data, Topics::kCameraLeftSide, Topics::kCameraLeftSideInfo, LocalMap::Frames::kCameraLeftSide);
                break;
            case DataLoader::CameraIndentifier::kCameraRightFront:
                cameraVisualizer_.drawRGBCameraFrameWithTopic(data, Topics::kCameraRightFront, Topics::kCameraRightFrontInfo, LocalMap::Frames::kCameraRightFront);
                break;
            case DataLoader::CameraIndentifier::kCameraRightSide:
                cameraVisualizer_.drawRGBCameraFrameWithTopic(data, Topics::kCameraRightSide, Topics::kCameraRightSideInfo, LocalMap::Frames::kCameraRightSide);
                break;
            default:
                context_.logger_.warning("Unexpected camera frame source when drawing RGB image");
                break;
        }
    }


    void VisualizationHandler::drawIRImage(std::shared_ptr<DataModels::CameraIrFrameDataModel> data) {

        if (!context_.getFunctionalityFlags().visualization_global_enable_) {return;}
        if (!context_.getFunctionalityFlags().ir_camera_visualization_) {return;}

        cameraVisualizer_.drawIRCameraFrameWithTopic(data, Topics::kCameraIr, Topics::kCameraIrInfo, LocalMap::Frames::kCameraIr);
    }


    void VisualizationHandler::drawVelocityData(rtl::Vector3D<double> speed) {

        if (!context_.getFunctionalityFlags().visualization_global_enable_) {return;}

        imuVisualizer_.drawImuData(speed, LocalMap::Frames::kImuFrame, Topics::kSpeedTopic);
    }

    void VisualizationHandler::drawImuData(const rtl::Vector3D<double> linAcc) {

        if (!context_.getFunctionalityFlags().visualization_global_enable_) {return;}
        if (!context_.getFunctionalityFlags().imu_visualization_) {return;}

        imuVisualizer_.drawImuData(linAcc, LocalMap::Frames::kImuFrame, Topics::kImuTopic);
    }

    void VisualizationHandler::drawImuAvgData(rtl::Vector3D<double> linAcc) {

        if (!context_.getFunctionalityFlags().visualization_global_enable_) {return;}
        if (!context_.getFunctionalityFlags().imu_visualization_) {return;}

        imuVisualizer_.drawImuData(linAcc, LocalMap::Frames::kImuFrame, Topics::kImuAvgTopic);
    }

    void VisualizationHandler::drawGnssPoseData(const std::shared_ptr<DataModels::GnssPoseDataModel> data) const {

        if (!context_.getFunctionalityFlags().visualization_global_enable_) {return;}
        if (!context_.getFunctionalityFlags().gnss_visualization_) {return;}

        gnssVisualizer_.drawGnssPose(data);
    }

    void VisualizationHandler::drawRawGnssTrajectory(const std::deque<DataModels::LocalPosition> &data) const {

        if (!context_.getFunctionalityFlags().visualization_global_enable_) {return;}
        if (!context_.getFunctionalityFlags().gnss_visualization_) {return;}

        trajectoryVisualizer_.drawRawTrajectory(data);
    }

    void VisualizationHandler::drawFilteredTrajectory(const std::deque<DataModels::LocalPosition> &data) const {

        if (!context_.getFunctionalityFlags().visualization_global_enable_) {return;}
        if (!context_.getFunctionalityFlags().gnss_visualization_) {return;}

        trajectoryVisualizer_.drawFilteredTrajectory(data);
    }

    void VisualizationHandler::drawImuGpsTrajectory(const std::deque<DataModels::LocalPosition> &data) const {

        if (!context_.getFunctionalityFlags().visualization_global_enable_) {return;}
        if (!context_.getFunctionalityFlags().imu_visualization_) {return;}

        trajectoryVisualizer_.drawImuGpsTrajectory(data);
    }

    void VisualizationHandler::updateOriginToRootTf(const DataModels::LocalPosition& pose) {

        if (!context_.getFunctionalityFlags().visualization_global_enable_) {return;}

        rtl::RigidTf3D<double> tf(pose.getOrientation(), pose.getPosition());
        tfTreeVisualizer_.updateOriginToRootTf(tf);
    }

    void VisualizationHandler::setCameraCalibParamsForCameraId(std::shared_ptr<DataModels::CameraCalibrationParamsDataModel> params, std::string frame) {
        cameraVisualizer_.setCameraParams(frame, params);
    }

    void VisualizationHandler::drawFrustumDetections(std::vector<std::shared_ptr<const DataModels::FrustumDetection>> detections) {
        if (!context_.getFunctionalityFlags().visualization_global_enable_) {return;}
        frustumVisualizer_.visualizeFrustumDetections(detections);
    }

    void VisualizationHandler::drawLidarDetection(std::vector<std::shared_ptr<const DataModels::LidarDetection>> detections) {
        if (!context_.getFunctionalityFlags().visualization_global_enable_) {return;}
        lidarVisualizer_.drawLidarDetections(detections, Topics::kLidarDetections, LocalMap::Frames::kImuFrame);
    }

    void VisualizationHandler::drawTelemetry(std::string telemetryText) {
        if (!context_.getFunctionalityFlags().visualization_global_enable_) {return;}
        telemetryVisualizer_.drawTelemetryAsText(telemetryText, LocalMap::Frames::kImuFrame, Topics::kTelemetryText);
    }

    void VisualizationHandler::drawRadarTiObjects(const std::vector<DataModels::RadarTiDataModel::Object>& objects) {

        if (!context_.getFunctionalityFlags().visualization_global_enable_) {return;}
        if (!context_.getFunctionalityFlags().radar_visualization_) {return;}

        radarVisualizer_.drawRadarDetectionsOnTopic(objects, Topics::kRadarTiObjects, LocalMap::Frames::kRadarTi);
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

    visualization_msgs::Marker VisualizationHandler::getSelfCube() const {
        visualization_msgs::Marker cube;
        cube.header.frame_id = LocalMap::Frames::kImuFrame;
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