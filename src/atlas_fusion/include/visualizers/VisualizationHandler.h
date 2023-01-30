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

#include "visualization_msgs/msg/marker.hpp"

#include "data_models/imu/ImuImuDataModel.h"
#include "data_models/camera/CameraFrameDataModel.h"
#include "data_models/camera/CameraIrFrameDataModel.h"
#include "data_models/DataModelTypes.h"
#include "data_models/local_map/LidarDetection.h"
#include "data_models/radar/RadarTiDataModel.h"

#include "Topics.h"

#include "LidarVisualizer.h"
#include "ImuVisualizer.h"
#include "CameraVisualizer.h"
#include "GnssVisualizer.h"
#include "TFVisualizer.h"
#include "FrustumVisualizer.h"
#include "RadarVisualizer.h"
#include "TrajectoryVisualizer.h"
#include "SensorStatusVisualizer.h"

#include "fail_check/AbstractFailChecker.h"


namespace AutoDrive::Visualizers {

    /**
     * The wrapper over the visualization backend. This class is the only way, how to Local Map could be visualized in
     * the given 3D environment (currently ROS backend)
     */
    class VisualizationHandler {

    public:

        /**
         * Constructor
         * @param node ros node reference
         * @param context global services container (timestamping, logging, etc.)
         */
        explicit VisualizationHandler(rclcpp::Node::SharedPtr & node, Context& context)
        : node_{node}
        , context_(context)
        , lidarVisualizer_(node, context)
        , imuVisualizer_(node, context)
        , cameraVisualizer_(node, context)
        , gnssVisualizer_(node, context)
        , tfTreeVisualizer_(node, context)
        , trajectoryVisualizer_(node, context)
        , frustumVisualizer_(node, context)
        , radarVisualizer_(node, context)
        , sensorStatusVisualizer_(node, context) {
            selfGlobalPublisher_ = node_->create_publisher<visualization_msgs::msg::Marker>(Topics::kSelfGlobal, 0);
            selfEgoPublisher_ = node_->create_publisher<visualization_msgs::msg::Marker>(Topics::kSelfEgo, 0);
        }

        /**
         * Drawing bounding box under the IMU sensor frame (represents mounted car)
         */
        void drawSelfGlobal() const;

        void drawSelfEgo() const;

        /**
         * Raw lidar scan data visualization
         * @param data point cloud to be visualized
         */
        void drawLidarData(const std::shared_ptr<DataModels::LidarScanDataModel>& data);

        /**
         * Render aggregated point cloud scans over the time in global coordinates
         * @param pc aggregated point cloud
         */
        void drawAggregatedPointCloudGlobal(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pc);

        /**
         * Render aggregated point cloud scans over the time in global coordinates
         * @param pc aggregated point cloud
         */
        void drawAggregatedPointCloudEgo(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pc);

        /**
         * Render aggregated point cloud from single laser
         * @param pc aggregated laser measurements
         */
        void drawAggregatedLasers(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pc);

        /**
         *
         * @param ls
         */
        void drawLidarApproximations(std::shared_ptr<std::vector<rtl::LineSegment3D<double>>> ls);

        /**
         *
         * @param ls
         */
        void drawLidarApproximationsRoad(std::shared_ptr<std::vector<rtl::LineSegment3D<double>>> ls);


        /**
         * Visualize point cloud aggregated during the entire mapping session
         * @param pc global point cloud map
         */
        void drawGlobalPointcloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pc);

        /**
         * Visualize point cloud cutout
         * @param pc bounded point cloud
         */
        void drawPointcloudCutout(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pc);


        /**
         * Render RGB image in the visialization engine
         * @param data image data
         */
        void drawRGBImage(const std::shared_ptr<DataModels::CameraFrameDataModel>& data);

        /**
         * Render IR image in the visialization engine
         * @param data image data
         */
        void drawIRImage(const std::shared_ptr<DataModels::CameraIrFrameDataModel>& data);


        /**
         * Render agents velocity vector in the visualization environment
         * @param velocity to be visualized
         */
        void drawVelocityData(const rtl::Vector3D<double>& speed);

        /**
         * Render current linear acceleration vector
         * @param linAcc 3D linear acceleration
         */
        void drawImuData(const rtl::Vector3D<double>& linAcc);

        /**
         * Render average linear acceleration vector
         * @param linAcc averaged 3D linear acceleration
         */
        void drawImuAvgData(const rtl::Vector3D<double>& linAcc);

        /**
         * Visualizes current WGS84 global position and heading as a text
         * @param data WGS84 position to be visualized
         */
        void drawGnssPoseData(const std::shared_ptr<DataModels::GnssPoseDataModel>& data) const;

        /**
         * Render raw gnss trajectory as a polyline
         * @param data dequeue of gnss positions
         */
        void drawRawGnssTrajectory(const std::deque<DataModels::LocalPosition> &data) const;

        /**
         * Render filtered gnss trajectory as a polyline in global coordinates
         * @param data dequeue of gnss positions
         */
        void drawFilteredTrajectory(const std::deque<DataModels::LocalPosition> &data) const;

        /**
         * Render raw imu gnss trajectory as a polyline
         * @param data dequeue of gnss positions
         */
        void drawImuGpsTrajectory(const std::deque<DataModels::LocalPosition> &data) const;

        /**
         * Updates current car position in the TF tree
         * @param pose current car to origin position
         */
        void updateOriginToRootTf(const DataModels::LocalPosition& pose);

        /**
         * Setups the camera calibration parameters for specific camera sensor. Used for projecting 3D scene into the
         * image plain in the visualization engine
         * @param params calibration params
         * @param frame camera sensor identifier
         */
        void setCameraCalibParamsForCameraId(DataModels::CameraCalibrationParamsDataModel& params, const FrameType& frame);

        /**
         * Draw frustum detections in 3D
         * @param detections frustum detections
         */
        void drawFrustumDetections(const std::vector<DataModels::FrustumDetection>& detections);

        /**
         * Draw lidar detections in 3D space
         * @param detections lidar detections
         */
        void drawLidarDetection(const std::vector<std::shared_ptr<DataModels::LidarDetection>>& detections);

        /**
         * Draws telemetry text message
         * @param telemetryText
         */
        void drawTelemetry(const std::string& telemetryText);

        /**
         * Draws radar ti scan data
         * @param vector of detected objects; x,y,z pose and radial velocity
         */
        void drawRadarTiObjects(const std::vector<DataModels::RadarTiDataModel::Object>& objects);

        /**
        * Draws sensor status text message
        * @param sensorStatus
        */
        void drawSensorStatus(const FailCheck::SensorStatus& sensorStatus, const FrameType& frameType);

        void drawEnvironmentalStatus(const std::string& environmentalStatus);

    private:

        rclcpp::Node::SharedPtr& node_;
        Context& context_;

        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr selfGlobalPublisher_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr selfEgoPublisher_;

        LidarVisualizer lidarVisualizer_;
        ImuVisualizer imuVisualizer_;
        CameraVisualizer cameraVisualizer_;
        GnssVisualizer gnssVisualizer_;
        TFVisualizer tfTreeVisualizer_;
        TrajectoryVisualizer trajectoryVisualizer_;
        FrustumVisualizer frustumVisualizer_;
        RadarVisualizer radarVisualizer_;

        SensorStatusVisualizer sensorStatusVisualizer_;

        [[nodiscard]] visualization_msgs::msg::Marker getSelfEgoCube() const;
        [[nodiscard]] visualization_msgs::msg::Marker getSelfGlobalCube() const;
    };

}
