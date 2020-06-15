#include "data_models/local_map/PointCloudBatch.h"

namespace AutoDrive::DataModels {


    std::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>> PointCloudBatch::getPoints() const {
        return std::make_shared<const pcl::PointCloud<pcl::PointXYZ>>(points_);
    }


    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> PointCloudBatch::getTransformedPoints() const {

        return transformPointsByTF(tf_, points_);
    }



    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> PointCloudBatch::getTransformedPointsWithAnotherTF(rtl::RigidTf3D<double>& tf) const {

        return transformPointsByTF( tf(tf_), points_ );

    }


    uint64_t PointCloudBatch::getTimestamp() const {
        return timestamp_;
    }


    size_t PointCloudBatch::getPointsSize() const {
        return points_.size();
    };


    std::string PointCloudBatch::getFrame() const {
        return referenceFrame_;
    };


    rtl::RigidTf3D<double> PointCloudBatch::getTF() const {
        return tf_;
    };



    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> PointCloudBatch::transformPointsByTF( const rtl::RigidTf3D<double>& tf, const pcl::PointCloud<pcl::PointXYZ>& pts ) const {

        auto output = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        output->reserve(pts.size());

        auto rotMat = tf.rotMat();
        Eigen::Affine3f pcl_tf = Eigen::Affine3f::Identity();
        pcl_tf(0,0) = static_cast<float>(rotMat(0, 0));
        pcl_tf(1,0) = static_cast<float>(rotMat(1, 0));
        pcl_tf(2,0) = static_cast<float>(rotMat(2, 0));
        pcl_tf(0,1) = static_cast<float>(rotMat(0, 1));
        pcl_tf(1,1) = static_cast<float>(rotMat(1, 1));
        pcl_tf(2,1) = static_cast<float>(rotMat(2, 1));
        pcl_tf(0,2) = static_cast<float>(rotMat(0, 2));
        pcl_tf(1,2) = static_cast<float>(rotMat(1, 2));
        pcl_tf(2,2) = static_cast<float>(rotMat(2, 2));
        pcl_tf.translation() << tf.trVecX(), tf.trVecY(), tf.trVecZ();
        pcl::transformPointCloud (pts, *output, pcl_tf);

        return output;
    }
}