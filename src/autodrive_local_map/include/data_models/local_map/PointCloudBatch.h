#pragma once

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include <rtl/Transformation3D.h>

namespace AutoDrive::DataModels {

    class PointCloudBatch {

    public:

        explicit PointCloudBatch(uint64_t ts, pcl::PointCloud<pcl::PointXYZ> points, std::string frame, rtl::Transformation3D<double> tf)
        : timestamp_{ts}
        , points_{points}
        , referenceFrame_{frame}
        , tf_{tf} {

        }

        uint64_t getTimestamp() {return timestamp_;};

        pcl::PointCloud<pcl::PointXYZ> getPoints() {return points_;};
        pcl::PointCloud<pcl::PointXYZ> getTransformedPoints() {

            pcl::PointCloud<pcl::PointXYZ> output;
            auto rotMat = tf_.rotQuaternion().rotMat();
            Eigen::Affine3f pcl_tf = Eigen::Affine3f::Identity();
            pcl_tf(0,0) = static_cast<float>(rotMat(0, 0)); pcl_tf(1,0) = static_cast<float>(rotMat(1, 0)); pcl_tf(2,0) = static_cast<float>(rotMat(2, 0));
            pcl_tf(0,1) = static_cast<float>(rotMat(0, 1)); pcl_tf(1,1) = static_cast<float>(rotMat(1, 1)); pcl_tf(2,1) = static_cast<float>(rotMat(2, 1));
            pcl_tf(0,2) = static_cast<float>(rotMat(0, 2)); pcl_tf(1,2) = static_cast<float>(rotMat(1, 2)); pcl_tf(2,2) = static_cast<float>(rotMat(2, 2));
            pcl_tf.translation() << tf_.trX(), tf_.trY(), tf_.trZ();
            pcl::transformPointCloud (points_, output, pcl_tf);

            std::cout << "Transforming: " << points_.at(0).x << " " << points_.at(0).y << " " << points_.at(0).z <<
                         " -> " << tf_.trX() << " " << tf_.trY() << " " << tf_.trZ() <<
                         " -> " << output.at(0).x << " " << output.at(0).y << " " << output.at(0).z << std::endl;
            return output;
        }

        std::string getFrame() {return referenceFrame_;};
        rtl::Transformation3D<double> getTF() {return tf_;};

    private:


        uint64_t timestamp_;
        pcl::PointCloud<pcl::PointXYZ> points_;
        std::string referenceFrame_;
        rtl::Transformation3D<double> tf_;
    };

}