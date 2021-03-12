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
#include "algorithms/pointcloud/PointCloudAggregator.h"
#include "algorithms/pointcloud/PointCloudExtrapolator.h"

#include "Context.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <data_models/local_map/LocalPosition.h>

#include <stdlib.h>
#include <time.h>


#define N 40
#define M 5
#define NOISE_AMP 0.00
//#define VISUALIZE

float getRandFloat() {
    return static_cast<float>( ( ( ( rand()%200 ) - 100 ) / 100.0 ) * NOISE_AMP );
}

pcl::PointCloud<pcl::PointXYZ> getTestData() {
    pcl::PointCloud<pcl::PointXYZ> output;

    for(int i = 0 ; i < N ; i++) {
        double step = 2*M_PI / N;
        double angle = step * i;
        output.push_back({(float)std::cos(angle) + getRandFloat() + 5, (float)-sin(angle) + getRandFloat() , getRandFloat()});
    }
    return output;
}


AtlasFusion::DataModels::LocalPosition getPoseDiff(AtlasFusion::DataModels::LocalPosition poseStart,
                                                   AtlasFusion::DataModels::LocalPosition poseEnd) {

    return AtlasFusion::DataModels::LocalPosition{{poseEnd.getPosition().x() - poseStart.getPosition().x(),
                                                 poseEnd.getPosition().y() - poseStart.getPosition().y(),
                                                 poseEnd.getPosition().z() - poseStart.getPosition().z()},
                                                  {poseEnd.getOrientation() * poseStart.getOrientation().inverted()},
                                                poseEnd.getTimestamp() - poseStart.getTimestamp()};
}

pcl::PointCloud<pcl::PointXYZ> distort(pcl::PointCloud<pcl::PointXYZ> pc, AtlasFusion::DataModels::LocalPosition distortion, size_t cycle) {

    pcl::PointCloud<pcl::PointXYZ> output;
    auto rot = distortion.getOrientation();

    for (size_t i = 0; i < N; i++) {

        float ratio = float(i) / N + cycle;

        float x_offset = (float) (distortion.getPosition().x()) * ratio;
        float y_offset = (float) (distortion.getPosition().y()) * ratio;
        float z_offset = (float) (distortion.getPosition().z()) * ratio;

        rtl::Quaternion<double> zero_rot{};
        rtl::Quaternion<double> partialRot{zero_rot.slerp(rot, ratio)};

        rtl::RigidTf3D <double> tf{partialRot, {x_offset, y_offset, z_offset}};
        tf = tf.inverted();
        auto rotMat = tf.rotMat();

        Eigen::Affine3f pcl_tf = Eigen::Affine3f::Identity();
        pcl_tf = Eigen::Affine3f::Identity();
        pcl_tf(0, 0) = static_cast<float>(rotMat(0, 0));
        pcl_tf(1, 0) = static_cast<float>(rotMat(1, 0));
        pcl_tf(2, 0) = static_cast<float>(rotMat(2, 0));
        pcl_tf(0, 1) = static_cast<float>(rotMat(0, 1));
        pcl_tf(1, 1) = static_cast<float>(rotMat(1, 1));
        pcl_tf(2, 1) = static_cast<float>(rotMat(2, 1));
        pcl_tf(0, 2) = static_cast<float>(rotMat(0, 2));
        pcl_tf(1, 2) = static_cast<float>(rotMat(1, 2));
        pcl_tf(2, 2) = static_cast<float>(rotMat(2, 2));
        pcl_tf.translation() << tf.trVecX(), tf.trVecY(), tf.trVecZ();

        pcl::PointCloud<pcl::PointXYZ> src;
        src.push_back(pc.at(i));
        pcl::PointCloud<pcl::PointXYZ> dest;
        pcl::transformPointCloud(src, dest, pcl_tf);

        output += dest;
    }

    return output;
}





class VisualizerAgg {

public:
    VisualizerAgg() {
        int argc = 0;
        char* argv[] = {0};

        ros::init(argc, argv, "autodrive_localmap");
        node_ = new ros::NodeHandle();
        pc_publisher_ = node_->advertise<sensor_msgs::PointCloud2>("pointcloud_aggregation", 0);
    }

    void publishPointcloud(pcl::PointCloud<pcl::PointXYZ>& pc) {
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(pc, msg);

        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "origin";
        msg.header.seq = 0;
        pc_publisher_.publish(msg);
    }
private:

    ros::NodeHandle* node_;
    ros::Publisher pc_publisher_;
};

#ifdef VISUALIZE
VisualizerAgg visualizer;
#endif


TEST(pointcloud_aggregation, init) {
    auto context = AtlasFusion::Context::getEmptyContext();
    AtlasFusion::Algorithms::PointCloudAggregator aggregator{context, 1.0};
}

TEST(pointcloud_aggregation, static_aggregation) {
    auto context = AtlasFusion::Context::getEmptyContext();
    AtlasFusion::Algorithms::PointCloudAggregator aggregator{context, 1.0};

    auto data = getTestData();

    for(size_t i = 0 ; i < data.size() ; i++) {

        pcl::PointCloud<pcl::PointXYZ> points;
        points.push_back({data.points.at(i).x, data.points.at(i).y, data.points.at(i).z});

        std::vector<std::shared_ptr<AtlasFusion::DataModels::PointCloudBatch>> batches;
        auto batch = std::make_shared<AtlasFusion::DataModels::PointCloudBatch> (0, points, "origin", rtl::RigidTf3D<double>::identity());
        batches.push_back(batch);

        aggregator.addPointCloudBatches(batches);
    }

    auto aggregatedPC = aggregator.getAggregatedPointCloud();
    EXPECT_EQ(aggregatedPC->size(), N);

#ifdef VISUALIZE
    visualizer.publishPointcloud(*aggregatedPC);
    visualizer.publishPointcloud(*aggregatedPC);
#endif
}


TEST(pointcloud_aggregation, multiple_static_aggregation) {
    auto context = AtlasFusion::Context::getEmptyContext();
    AtlasFusion::Algorithms::PointCloudAggregator aggregator{context, 1.0};

    for(size_t j = 0 ; j < M ; j++) {
        auto data = getTestData();

        for(size_t i = 0 ; i < data.size() ; i++) {

            pcl::PointCloud<pcl::PointXYZ> points;
            points.push_back({data.points.at(i).x, data.points.at(i).y, data.points.at(i).z});

            std::vector<std::shared_ptr<AtlasFusion::DataModels::PointCloudBatch>> batches;
            auto batch = std::make_shared<AtlasFusion::DataModels::PointCloudBatch> (j * 1e9, points, "origin", rtl::RigidTf3D<double>::identity());
            batches.push_back(batch);

            aggregator.addPointCloudBatches(batches);
        }
    }

    auto aggregatedPC = aggregator.getAggregatedPointCloud();
    EXPECT_EQ(aggregatedPC->size(), N*M);


#ifdef VISUALIZE
    visualizer.publishPointcloud(*aggregatedPC);
    visualizer.publishPointcloud(*aggregatedPC);
    visualizer.publishPointcloud(*aggregatedPC);
    visualizer.publishPointcloud(*aggregatedPC);
    visualizer.publishPointcloud(*aggregatedPC);
    visualizer.publishPointcloud(*aggregatedPC);
    visualizer.publishPointcloud(*aggregatedPC);
#endif
}


TEST(pointcloud_aggregation, points_filtration) {
    auto context = AtlasFusion::Context::getEmptyContext();
    AtlasFusion::Algorithms::PointCloudAggregator aggregator{context, 0.99};

    for(size_t j = 0 ; j < M ; j++) {
        auto data = getTestData();

        for(size_t i = 0 ; i < data.size() ; i++) {

            pcl::PointCloud<pcl::PointXYZ> points;
            points.push_back({data.points.at(i).x, data.points.at(i).y, data.points.at(i).z});

            std::vector<std::shared_ptr<AtlasFusion::DataModels::PointCloudBatch>> batches;
            auto batch = std::make_shared<AtlasFusion::DataModels::PointCloudBatch> (j * 1e9, points, "origin", rtl::RigidTf3D<double>::identity());
            batches.push_back(batch);

            auto ts = static_cast<uint64_t>(j*1e9);
            aggregator.filterOutBatches(ts);
            aggregator.addPointCloudBatches(batches);
        }
    }

    auto aggregatedPC = aggregator.getAggregatedPointCloud();
    EXPECT_EQ(aggregatedPC->size(), N);

#ifdef VISUALIZE
    visualizer.publishPointcloud(*aggregatedPC);
    visualizer.publishPointcloud(*aggregatedPC);
#endif
}



TEST(pointcloud_aggregation, forward_movement) {
    auto context = AtlasFusion::Context::getEmptyContext();
    AtlasFusion::Algorithms::PointCloudAggregator aggregator{context, 0.99};
    AtlasFusion::Algorithms::PointCloudExtrapolator extrapolator(context, N);

    AtlasFusion::DataModels::LocalPosition startPose {{0, 0, 0}, {}, 0};
    AtlasFusion::DataModels::LocalPosition oneStepPose {{1, 0, 0}, {}, uint64_t(0.1e9)};
    auto poseDiff = getPoseDiff(startPose, oneStepPose);

    pcl::PointCloud<pcl::PointXYZ> data;
    pcl::PointCloud<pcl::PointXYZ> distData;
    pcl::PointCloud<pcl::PointXYZ> undistData;

    for(size_t i = 0 ; i < M ; i ++) {
        auto tmp = getTestData();
        auto distTmp = distort(tmp, poseDiff, i);

        auto currentPose = startPose;
        for(size_t j = 0 ; j < i ; j++) {
            currentPose = AtlasFusion::DataModels::LocalPosition{
                    currentPose.getPosition() + poseDiff.getPosition(),
                    currentPose.getOrientation() * poseDiff.getOrientation(),
                    currentPose.getTimestamp() + poseDiff.getTimestamp()
            };
        }

        auto batches = extrapolator.splitPointCloudToBatches(std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(distTmp), currentPose, poseDiff, rtl::RigidTf3D<double>::identity());
        aggregator.addPointCloudBatches(batches);

        data += tmp;
        distData += distTmp;
    }


    auto aggregatedPC = aggregator.getAggregatedPointCloud();
    EXPECT_EQ(aggregatedPC->size(), N*M);

#ifdef VISUALIZE
    visualizer.publishPointcloud(data);
    visualizer.publishPointcloud(data);
    visualizer.publishPointcloud(distData);
    visualizer.publishPointcloud(distData);
    visualizer.publishPointcloud(*aggregatedPC);
    visualizer.publishPointcloud(*aggregatedPC);
#endif
}


TEST(pointcloud_aggregation, rotation_movement) {
    auto context = AtlasFusion::Context::getEmptyContext();
    AtlasFusion::Algorithms::PointCloudAggregator aggregator{context, 0.99};
    AtlasFusion::Algorithms::PointCloudExtrapolator extrapolator(context, N);

    AtlasFusion::DataModels::LocalPosition startPose {{0, 0, 0}, {}, 0};
    AtlasFusion::DataModels::LocalPosition oneStepPose {{0, 0, 0}, {0.707, 0, 0, 0.707}, uint64_t(0.1e9)};
    auto poseDiff = getPoseDiff(startPose, oneStepPose);

    pcl::PointCloud<pcl::PointXYZ> data;
    pcl::PointCloud<pcl::PointXYZ> distData;
    pcl::PointCloud<pcl::PointXYZ> undistData;

    for(size_t i = 0 ; i < M ; i ++) {
        auto tmp = getTestData();
        auto distTmp = distort(tmp, poseDiff, i);

        auto currentPose = startPose;
        for(size_t j = 0 ; j < i ; j++) {
            currentPose = AtlasFusion::DataModels::LocalPosition{
                    currentPose.getPosition() + poseDiff.getPosition(),
                    currentPose.getOrientation() * poseDiff.getOrientation(),
                    currentPose.getTimestamp() + poseDiff.getTimestamp()
            };
        }

        auto batches = extrapolator.splitPointCloudToBatches(std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(distTmp), currentPose, poseDiff, {});
        aggregator.addPointCloudBatches(batches);

        for (const auto& batch : batches) {
            undistData += *(batch->getTransformedPoints());
        }

        data += tmp;
        distData += distTmp;
    }


    auto aggregatedPC = aggregator.getAggregatedPointCloud();
    EXPECT_EQ(aggregatedPC->size(), N*M);


#ifdef VISUALIZE
    visualizer.publishPointcloud(data);
    visualizer.publishPointcloud(data);
    visualizer.publishPointcloud(distData);
    visualizer.publishPointcloud(distData);
    visualizer.publishPointcloud(undistData);
    visualizer.publishPointcloud(undistData);
    visualizer.publishPointcloud(*aggregatedPC);
    visualizer.publishPointcloud(*aggregatedPC);
#endif
}



TEST(pointcloud_aggregation, translation_and_rotation) {
    auto context = AtlasFusion::Context::getEmptyContext();
    AtlasFusion::Algorithms::PointCloudAggregator aggregator{context, 0.99};
    AtlasFusion::Algorithms::PointCloudExtrapolator extrapolator(context, N);

    AtlasFusion::DataModels::LocalPosition startPose {{0, 0, 0}, {}, 0};
    AtlasFusion::DataModels::LocalPosition oneStepPose {{1, 1, 0}, {0.707, 0, 0, 0.707}, uint64_t(0.1e9)};
    auto poseDiff = getPoseDiff(startPose, oneStepPose);

    pcl::PointCloud<pcl::PointXYZ> data;
    pcl::PointCloud<pcl::PointXYZ> distData;
    pcl::PointCloud<pcl::PointXYZ> undistData;

    for(size_t i = 0 ; i < M ; i ++) {
        auto tmp = getTestData();
        auto distTmp = distort(tmp, poseDiff, i);

        auto currentPose = startPose;
        for(size_t j = 0 ; j < i ; j++) {
            currentPose = AtlasFusion::DataModels::LocalPosition{
                    currentPose.getPosition() + poseDiff.getPosition(),
                    currentPose.getOrientation() * poseDiff.getOrientation(),
                    currentPose.getTimestamp() + poseDiff.getTimestamp()
            };
        }
//
//        std::cout << ""

        auto batches = extrapolator.splitPointCloudToBatches(std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(distTmp), currentPose, poseDiff, {});
        aggregator.addPointCloudBatches(batches);

        for (const auto& batch : batches) {
            undistData += *(batch->getTransformedPoints());
        }

        data += tmp;
        distData += distTmp;

#ifdef VISUALIZE
        visualizer.publishPointcloud(undistData);
#endif
    }


    auto aggregatedPC = aggregator.getAggregatedPointCloud();
    EXPECT_EQ(aggregatedPC->size(), N*M);


#ifdef VISUALIZE
    visualizer.publishPointcloud(data);
    visualizer.publishPointcloud(data);
    visualizer.publishPointcloud(distData);
    visualizer.publishPointcloud(distData);
    visualizer.publishPointcloud(undistData);
    visualizer.publishPointcloud(undistData);
    visualizer.publishPointcloud(*aggregatedPC);
    visualizer.publishPointcloud(*aggregatedPC);
#endif
}



int main(int argc, char **argv){

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}