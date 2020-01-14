#pragma once

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include <rtl/Transformation3D.h>

namespace AutoDrive::DataModels {

    class PointCloudBatch {

    public:

        PointCloudBatch() = delete;

        explicit PointCloudBatch(uint64_t ts, pcl::PointCloud<pcl::PointXYZ> points, std::string frame, rtl::Transformation3D<double> tf)
        : timestamp_{ts}
        , points_{points}
        , referenceFrame_{frame}
        , tf_{tf} {

        }

        uint64_t getTimestamp() const {return timestamp_;};

        std::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>> getPoints() const {return std::make_shared<const pcl::PointCloud<pcl::PointXYZ>>(points_);};
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> getTransformedPoints() const {

            auto output = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
            auto rotMat = tf_.rotQuaternion().rotMat();
            Eigen::Affine3f pcl_tf = Eigen::Affine3f::Identity();
            pcl_tf(0,0) = static_cast<float>(rotMat(0, 0)); pcl_tf(1,0) = static_cast<float>(rotMat(1, 0)); pcl_tf(2,0) = static_cast<float>(rotMat(2, 0));
            pcl_tf(0,1) = static_cast<float>(rotMat(0, 1)); pcl_tf(1,1) = static_cast<float>(rotMat(1, 1)); pcl_tf(2,1) = static_cast<float>(rotMat(2, 1));
            pcl_tf(0,2) = static_cast<float>(rotMat(0, 2)); pcl_tf(1,2) = static_cast<float>(rotMat(1, 2)); pcl_tf(2,2) = static_cast<float>(rotMat(2, 2));
            pcl_tf.translation() << tf_.trX(), tf_.trY(), tf_.trZ();
            pcl::transformPointCloud (points_, *output, pcl_tf);
            return output;
        }

        size_t getPointsSize() const { return points_.size(); };

        std::string getFrame() const { return referenceFrame_; };
        rtl::Transformation3D<double> getTF() const { return tf_; };

    private:

        uint64_t timestamp_;
        pcl::PointCloud<pcl::PointXYZ> points_;
        std::string referenceFrame_;
        rtl::Transformation3D<double> tf_;
    };

}