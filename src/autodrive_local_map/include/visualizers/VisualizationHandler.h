#pragma once

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <memory>

#include <rtl/Frustum.h>

#include "data_models/imu/ImuImuDataModel.h"
#include "data_models/camera/CameraFrameDataModel.h"
#include "data_models/camera/CameraIrFrameDataModel.h"
#include "data_models/DataModelTypes.h"

#include "Topics.h"
#include "Context.h"

#include "LidarVisualizer.h"
#include "ImuVisualizer.h"
#include "CameraVisualizer.h"
#include "GnssVisualizer.h"
#include "TFVisualizer.h"
#include "FrustumVisualizer.h"

#include "TrajectoryVisualizer.h"

namespace AutoDrive::Visualizers {

    class VisualizationHandler {

    public:

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
        , cameraParams_{} {
            testCubePublisher_ = node_.advertise<visualization_msgs::Marker>( Topics::kTestCubeTopic, 0 );
        }

        void drawTestingCube() const;

        void drawLidarData(std::shared_ptr<DataModels::LidarScanDataModel>) const;
        void drawImuData(rtl::Vector3D<double> linAcc) const;
        void drawRGBImage(std::shared_ptr<DataModels::CameraFrameDataModel>) const;
        void drawIRImage(std::shared_ptr<DataModels::CameraIrFrameDataModel>) const;
        void drawGnssPoseData(std::shared_ptr<DataModels::GnssPoseDataModel>) const;

        void drawRawGnssTrajectory(const std::deque<DataModels::LocalPosition> &data) const;
        void drawFilteredTrajectory(const std::deque<DataModels::LocalPosition> &data) const;
        void drawImuGpsTrajectory(const std::deque<DataModels::LocalPosition> &data) const;

        void updateOriginToRootTf(const DataModels::LocalPosition& pose);

        void setCameraCalibParamsForCameraId(std::shared_ptr<DataModels::CameraCalibrationParamsDataModel> params, DataLoader::CameraIndentifier id);

        void drawFrustumDetections(std::vector<std::shared_ptr<DataModels::FrustumDetection>> detections);

    private:

        ros::NodeHandle& node_;
        Context& context_;

        ros::Publisher testCubePublisher_;

        LidarVisualizer lidarVisualizer_;
        ImuVisualizer imuVisualizer_;
        CameraVisualizer cameraVisualizer_;
        GnssVisualizer gnssVisualizer_;
        TFVisualizer tfTreeVisualizer_;
        TrajectoryVisualizer trajectoryVisualizer_;
        FrustumVisualizer frustumVisualizer_;

        std::map<DataLoader::CameraIndentifier, std::shared_ptr<DataModels::CameraCalibrationParamsDataModel>> cameraParams_;

        visualization_msgs::Marker getTestCube() const;
    };

}
