#include <gtest/gtest.h>
#include "algorithms/pointcloud/PointCloudExtrapolator.h"

#include "Context.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#define N 40



pcl::PointCloud<pcl::PointXYZ> getTestData() {
    pcl::PointCloud<pcl::PointXYZ> output;

    for(int i = 0 ; i < N ; i++) {
        double step = 2*M_PI / N;
        double angle = step * i;
        output.push_back({(float)std::cos(angle) + 2, (float)-sin(angle), 0});
    }
    return output;
}

pcl::PointCloud<pcl::PointXYZ> distort(pcl::PointCloud<pcl::PointXYZ> pc, AutoDrive::DataModels::LocalPosition distortion) {

    pcl::PointCloud<pcl::PointXYZ> output;
    auto rot = distortion.getOrientation();
    for(size_t i = 0 ; i < N ; i++) {

        float ratio = float(i)/N;
        float x_offset = -(float)(distortion.getPosition().x()) * ratio;
        float y_offset = -(float)(distortion.getPosition().y()) * ratio;
        float z_offset = -(float)(distortion.getPosition().z()) * ratio;

        rtl::Quaternion<double> zero_rot{};
        rtl::Quaternion<double> partialRot{zero_rot.eigenQuat().slerp(ratio, rot.eigenQuat())};
        auto rotMat = partialRot.inverted().rotMat();

        Eigen::Affine3f pcl_tf = Eigen::Affine3f::Identity();
        pcl_tf(0,0) = static_cast<float>(rotMat(0, 0)); pcl_tf(1,0) = static_cast<float>(rotMat(1, 0)); pcl_tf(2,0) = static_cast<float>(rotMat(2, 0));
        pcl_tf(0,1) = static_cast<float>(rotMat(0, 1)); pcl_tf(1,1) = static_cast<float>(rotMat(1, 1)); pcl_tf(2,1) = static_cast<float>(rotMat(2, 1));
        pcl_tf(0,2) = static_cast<float>(rotMat(0, 2)); pcl_tf(1,2) = static_cast<float>(rotMat(1, 2)); pcl_tf(2,2) = static_cast<float>(rotMat(2, 2));
        pcl_tf.translation() << x_offset, y_offset, z_offset;

        pcl::PointCloud<pcl::PointXYZ> src;
        src.push_back(pc.at(i));
        pcl::PointCloud<pcl::PointXYZ> dest;
        pcl::transformPointCloud (src, dest, pcl_tf);
        output += dest;
    }
    return output;
}

AutoDrive::DataModels::LocalPosition getPoseDiff(AutoDrive::DataModels::LocalPosition poseStart,
                                                 AutoDrive::DataModels::LocalPosition poseEnd) {

    return AutoDrive::DataModels::LocalPosition{{poseEnd.getPosition().x() - poseStart.getPosition().x(),
                                                 poseEnd.getPosition().y() - poseStart.getPosition().y(),
                                                 poseEnd.getPosition().z() - poseStart.getPosition().z()},
                                                {poseEnd.getOrientation() * poseStart.getOrientation().inverted()},
                                                poseEnd.getTimestamp() - poseStart.getTimestamp()};
}




class Visualizer {

public:
    Visualizer() {
        int argc = 0;
        char* argv[] = {0};
        ros::init(argc, argv, "autodrive_localmap");
        node_ = new ros::NodeHandle();
        pc_publisher_ = node_->advertise<sensor_msgs::PointCloud2>("pointcloud_extrapolation", 10);
    }

    void publishPointcloud(pcl::PointCloud<pcl::PointXYZ> pc) {
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(pc, msg);

        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "origin";
        msg.header.seq = seq;
        pc_publisher_.publish(msg);
    }
private:

    ros::NodeHandle* node_;
    ros::Publisher pc_publisher_;
    size_t seq = 0;
};

Visualizer visualizer;

TEST(pointcloud_extrapolation, init) {
    auto context = AutoDrive::Context::getEmptyContext();
    AutoDrive::Algorithms::PointCloudExtrapolator extrapolator{context, N};
}


TEST(pointcloud_extrapolation, forward_movement) {
    auto context = AutoDrive::Context::getEmptyContext();
    AutoDrive::Algorithms::PointCloudExtrapolator extrapolator{context, N};


    auto data = getTestData();
    std::cout << " ************************ " << std::endl;
    for(int i = 0 ; i < N ; i++) {
        std::cout << "Point " << i+1 << ": " << data.at(i).x << " " << data.at(i).y << " " << data.at(i).z << std::endl;
    }

    AutoDrive::DataModels::LocalPosition startPose {{0,0,0}, {}, 0};
    AutoDrive::DataModels::LocalPosition endPose {{1,0,0}, {}, 0};
    auto poseDiff = getPoseDiff(startPose, endPose);


    std::cout << "Pose start: " << startPose.getPosition().x() << " " << startPose.getPosition().y() << " " << startPose.getPosition().z() << " | "
              << startPose.getOrientation().x() << " " << startPose.getOrientation().y() << " " << startPose.getOrientation().z() << " " << startPose.getOrientation().w() << std::endl;
    std::cout << "Pose end: " << endPose.getPosition().x() << " " << endPose.getPosition().y() << " " << endPose.getPosition().z() << " | "
              << endPose.getOrientation().x() << " " << endPose.getOrientation().y() << " " << endPose.getOrientation().z() << " " << endPose.getOrientation().w() << std::endl;
    auto startPlusDiff=AutoDrive::DataModels::LocalPosition{{startPose.getPosition().x() + poseDiff.getPosition().x(),
                                                             startPose.getPosition().y() + poseDiff.getPosition().y(),
                                                             startPose.getPosition().z() + poseDiff.getPosition().z()},
                                                            {startPose.getOrientation() * poseDiff.getOrientation()},
                                                            startPose.getTimestamp() + poseDiff.getTimestamp()};
    std::cout << "Pose Start * pose diff: " << startPlusDiff.getPosition().x() << " " << startPlusDiff.getPosition().y() << " " << startPlusDiff.getPosition().z() << " | "
              << startPlusDiff.getOrientation().x() << " " << startPlusDiff.getOrientation().y() << " " << startPlusDiff.getOrientation().z() << " " << startPlusDiff.getOrientation().w() << std::endl;


    auto data_dist = distort(data, poseDiff);

    std::cout << " ************************ " << std::endl;
    for(int i = 0 ; i < N ; i++) {
        std::cout << "Distorted " << i+1 << ": " << data_dist.at(i).x << " " << data_dist.at(i).y << " " << data_dist.at(i).z << std::endl;
    }


    auto batches = extrapolator.splitPointCloudToBatches(
            std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(data_dist), endPose, poseDiff, {});

    pcl::PointCloud<pcl::PointXYZ> undist_data;
    for (const auto& batch : batches) {
        undist_data += batch->getTransformedPoints();
    }

    std::cout << " ************************ " << std::endl;
    for(int i = 0 ; i < N ; i++) {
        std::cout << "Back Transformed " << i+1 << ": " << undist_data.at(i).x << " " << undist_data.at(i).y << " " << undist_data.at(i).z << std::endl;
    }

    visualizer.publishPointcloud(data);
    visualizer.publishPointcloud(data_dist);
    visualizer.publishPointcloud(undist_data);
}


TEST(pointcloud_extrapolation, rotation_left) {
    auto context = AutoDrive::Context::getEmptyContext();
    AutoDrive::Algorithms::PointCloudExtrapolator extrapolator{context, N};

    auto data = getTestData();


    AutoDrive::DataModels::LocalPosition startPose {{0,0,0}, {1,0,0,0}, 0};
    AutoDrive::DataModels::LocalPosition endPose {{0,0,0}, {0.707, 0, 0, 0.707}, 0};
    auto poseDiff = getPoseDiff(startPose, endPose);
    auto data_dist = distort(data, poseDiff);


    auto batches = extrapolator.splitPointCloudToBatches(
            std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(data_dist),
            AutoDrive::DataModels::LocalPosition{{},{},0},
            poseDiff,
            {});

    pcl::PointCloud<pcl::PointXYZ> undist_data;
    for (const auto& batch : batches) {
        undist_data += batch->getTransformedPoints();
    }

    visualizer.publishPointcloud(data);
    visualizer.publishPointcloud(data_dist);
    visualizer.publishPointcloud(undist_data);
}


TEST(pointcloud_extrapolation, rotation_and_translation) {
    auto context = AutoDrive::Context::getEmptyContext();
    AutoDrive::Algorithms::PointCloudExtrapolator extrapolator{context, N};

    auto data = getTestData();


    AutoDrive::DataModels::LocalPosition startPose {{0,0,0}, {1,0,0,0}, 0};
    AutoDrive::DataModels::LocalPosition endPose {{1,0,0}, {0.707, 0, 0, 0.707}, 0};
    auto poseDiff = getPoseDiff(startPose, endPose);
    auto data_dist = distort(data, poseDiff);



    auto batches = extrapolator.splitPointCloudToBatches(
        std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(data_dist),
        AutoDrive::DataModels::LocalPosition{{},{},0},
        poseDiff,
        {});

    pcl::PointCloud<pcl::PointXYZ> undist_data;
    for (const auto& batch : batches) {
        undist_data += batch->getTransformedPoints();
    }

    visualizer.publishPointcloud(data);
    visualizer.publishPointcloud(data_dist);
    visualizer.publishPointcloud(undist_data);
}


int main(int argc, char **argv){

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}