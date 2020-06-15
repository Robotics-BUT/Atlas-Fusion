#include <gtest/gtest.h>
#include "algorithms/DepthMap.h"


const cv::Mat getIntrinsic() {
    return (cv::Mat_<double> (3,3) <<
                                   1.3763974496214614e+03, 0.0000000000000000e+00, 9.5923661859185984e+02,
            0.0000000000000000e+00, 1.3771721669205588e+03, 5.7674337567054886e+02,
            0.0000000000000000e+00, 0.0000000000000000e+00, 1.0000000000000000e+00
    );
}

const cv::Mat getDistortion() {
    return (cv::Mat_<double> (1,5) <<
                                   -1.7245433170404437e-01, 1.1239016157509915e-01, -1.5253720111333031e-04, 3.8487261087130094e-04, -2.0770262391598841e-02
    );
}

const cv::Mat getDistortionNull() {
    return (cv::Mat_<double> (1,5) <<
                                   0,0,0,0,0
    );
}

rtl::RigidTf3D<double> getTF(){
    return rtl::RigidTf3D<double>{rtl::Quaternion<double>::identity(), {0.0, 0.0, 0.0}};
}

std::shared_ptr<AutoDrive::Algorithms::Projector> getProjector() {
    auto tf = getTF();
    return std::make_shared<AutoDrive::Algorithms::Projector>(getIntrinsic(), getDistortion(), tf);
}


TEST(depth_map_test, initialization) {
    auto context = AutoDrive::Context::getEmptyContext();
    auto depthMap = AutoDrive::Algorithms::DepthMap(context);
}

TEST(depth_map_test, add_projector) {

    auto context = AutoDrive::Context::getEmptyContext();
    auto depthMap = AutoDrive::Algorithms::DepthMap(context);

    depthMap.addProjector(getProjector(), AutoDrive::DataLoader::CameraIndentifier::kCameraLeftFront);
}


TEST(depth_map_test, on_lidar_data) {

//    auto context = AutoDrive::Context::getValidContext();
//    auto depthMap = AutoDrive::Algorithms::DepthMap(context);
//    depthMap.addProjector(getProjector(), AutoDrive::DataLoader::CameraIndentifier::kCameraLeftFront);
//
//    auto scan = std::make_shared<AutoDrive::DataModels::LidarScanDataModel>(0, AutoDrive::DataLoader::LidarIdentifier::kRightLidar, "", 0);
//    scan->addPointToScan(pcl::PointXYZ{10,0,0});
//
//    depthMap.onNewLidarData(scan);
}


int main(int argc, char **argv){

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}