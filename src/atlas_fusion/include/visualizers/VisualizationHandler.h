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

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <memory>

#include <rtl/Core.h>

#include "data_models/imu/ImuImuDataModel.h"
#include "data_models/camera/CameraFrameDataModel.h"
#include "data_models/camera/CameraIrFrameDataModel.h"
#include "data_models/DataModelTypes.h"
#include "data_models/local_map/LidarDetection.h"
#include "data_models/radar/RadarTiDataModel.h"

#include "Topics.h"
#include "Context.h"

#include "LidarVisualizer.h"
#include "ImuVisualizer.h"
#include "CameraVisualizer.h"
#include "GnssVisualizer.h"
#include "TFVisualizer.h"
#include "FrustumVisualizer.h"
#include "TelemetryVisualizer.h"
#include "RadarVisualizer.h"

#include "TrajectoryVisualizer.h"

namespace AtlasFusion::Visualizers {

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
        explicit VisualizationHandler(ros::NodeHandle& node, Context& context)
        : node_{node}
        , context_(context)
        , lidarVisualizer_(node, context)
        , imuVisualizer_(node, context)
        , cameraVisualizer_(node, context)
        , gnssVisualizer_(node, context)
        , tfTreeVisualizer_(node, context)
        , trajectoryVisualizer_(node, context)
        , frustumVisualizer_(node, context)
        , telemetryVisualizer_(node, context)
        , radarVisualizer_(node, context) {
            testCubePublisher_ = node_.advertise<visualization_msgs::Marker>( Topics::kTestCubeTopic, 0 );
            selfPublisher_ = node_.advertise<visualization_msgs::Marker>( Topics::kSelf, 0);
        }

        /**
         * Simple cube test draw in the origin.
         */
        void drawTestingCube() const;

        /**
         * Drawing bounding box under the IMU sensor frame (represents mounted car)
         */
        void drawSelf() const;

        /**
         * Raw lidar scan data visualization
         * @param data point cloud to be visualized
         */
        void drawLidarData(std::shared_ptr<DataModels::LidarScanDataModel> data);

        /**
         * Render aggregated point cloud scans over the time
         * @param pc aggregated point cloud
         */
        void drawAggregatedPointcloud(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pc);

        /**
         * Render aggregated point cloud from single laser
         * @param pc aggregated laser measurements
         */
        void drawAggregatedLasers(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pc);

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
        void drawGlobalPointcloud(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pc);

        /**
         * Visualize point cloud cutout
         * @param pc bounded point cloud
         */
        void drawPointcloudCutout(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pc);


        /**
         * Render RGB image in the visialization engine
         * @param data image data
         */
        void drawRGBImage(std::shared_ptr<DataModels::CameraFrameDataModel> data);

        /**
         * Render IR image in the visialization engine
         * @param data image data
         */
        void drawIRImage(std::shared_ptr<DataModels::CameraIrFrameDataModel> data);


        /**
         * Render agents velocity vector in the visualization environment
         * @param velocity to be visualized
         */
        void drawVelocityData(rtl::Vector3D<double> speed);

        /**
         * Render current linear acceleration vector
         * @param linAcc 3D linear acceleration
         */
        void drawImuData(rtl::Vector3D<double> linAcc);

        /**
         * Render average linear acceleration vector
         * @param linAcc averaged 3D linear acceleration
         */
        void drawImuAvgData(rtl::Vector3D<double> linAcc);

        /**
         * Visualizes current WGS84 global position and heading as a text
         * @param data WGS84 position to be visualized
         */
        void drawGnssPoseData(std::shared_ptr<DataModels::GnssPoseDataModel> data) const;

        /**
         * Render raw gnss trajectory as a polyline
         * @param data dequeue of gnss positions
         */
        void drawRawGnssTrajectory(const std::deque<DataModels::LocalPosition> &data) const;

        /**
         * Render filtered gnss trajectory as a polyline
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
         * @param params caibration params
         * @param frame camera sensor identifier
         */
        void setCameraCalibParamsForCameraId(std::shared_ptr<DataModels::CameraCalibrationParamsDataModel> params, std::string frame);

        /**
         * Draw frustum detections in 3D
         * @param detections frustum detections
         */
        void drawFrustumDetections(std::vector<std::shared_ptr<const DataModels::FrustumDetection>> detections);

        /**
         * Draw lidar detections in 3D space
         * @param detections lidar detections
         */
        void drawLidarDetection(std::vector<std::shared_ptr<const DataModels::LidarDetection>> detections);

        /**
         * Draws telemetry text message
         * @param telemetryText
         */
        void drawTelemetry(std::string telemetryText);

        /**
         * Draws radar ti scan data
         * @param vector of detected objects; x,y,z pose and radial velocity
         */
        void drawRadarTiObjects(const std::vector<DataModels::RadarTiDataModel::Object>& objects);

    private:

        ros::NodeHandle& node_;
        Context& context_;

        ros::Publisher testCubePublisher_;
        ros::Publisher selfPublisher_;

        LidarVisualizer lidarVisualizer_;
        ImuVisualizer imuVisualizer_;
        CameraVisualizer cameraVisualizer_;
        GnssVisualizer gnssVisualizer_;
        TFVisualizer tfTreeVisualizer_;
        TrajectoryVisualizer trajectoryVisualizer_;
        FrustumVisualizer frustumVisualizer_;
        TelemetryVisualizer telemetryVisualizer_;
        RadarVisualizer radarVisualizer_;

        visualization_msgs::Marker getTestCube() const;
        visualization_msgs::Marker getSelfCube() const;
    };

}
