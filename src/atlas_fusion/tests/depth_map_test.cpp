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

std::shared_ptr<AtlasFusion::Algorithms::Projector> getProjector() {
    auto tf = getTF();
    return std::make_shared<AtlasFusion::Algorithms::Projector>(getIntrinsic(), getDistortion(), tf);
}


TEST(depth_map_test, initialization) {
    auto context = AtlasFusion::Context::getEmptyContext();
    auto depthMap = AtlasFusion::Algorithms::DepthMap(context);
}

TEST(depth_map_test, add_projector) {

    auto context = AtlasFusion::Context::getEmptyContext();
    auto depthMap = AtlasFusion::Algorithms::DepthMap(context);

    depthMap.addProjector(getProjector(), AtlasFusion::DataLoader::CameraIndentifier::kCameraLeftFront);
}


TEST(depth_map_test, on_lidar_data) {

//    auto context = AtlasFusion::Context::getValidContext();
//    auto depthMap = AtlasFusion::Algorithms::DepthMap(context);
//    depthMap.addProjector(getProjector(), AtlasFusion::DataLoader::CameraIndentifier::kCameraLeftFront);
//
//    auto scan = std::make_shared<AtlasFusion::DataModels::LidarScanDataModel>(0, AtlasFusion::DataLoader::LidarIdentifier::kRightLidar, "", 0);
//    scan->addPointToScan(pcl::PointXYZ{10,0,0});
//
//    depthMap.onNewLidarData(scan);
}


int main(int argc, char **argv){

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}