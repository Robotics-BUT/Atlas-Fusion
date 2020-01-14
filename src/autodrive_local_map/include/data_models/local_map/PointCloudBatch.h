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
        , points_{std::move(points)}
        , referenceFrame_{std::move(frame)}
        , tf_{tf} {

        }

        std::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>> getPoints() const;
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> getTransformedPoints() const;
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> getTransformedPointsWithAnotherTF(rtl::Transformation3D<double> tf) const;

        uint64_t getTimestamp() const;
        size_t getPointsSize() const;
        std::string getFrame() const;
        rtl::Transformation3D<double> getTF() const;

    private:
        uint64_t timestamp_;
        pcl::PointCloud<pcl::PointXYZ> points_;
        std::string referenceFrame_;
        rtl::Transformation3D<double> tf_;

        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> transformPointsByTF( rtl::Transformation3D<double> tf ) const;
    };

}