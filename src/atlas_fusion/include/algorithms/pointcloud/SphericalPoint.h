#pragma once

#include <rtl/Core.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

namespace AtlasFusion::Algorithms {

    template <class T>
    class SphericalPoint {

    public:
        SphericalPoint(T radius, T theta, T phi) :
                radius_{radius},
                theta_{theta},
                phi_{phi} {

        }

        T radius() const { return radius_; }
        T theta() const { return theta_; }
        T phi() const { return phi_; }

        static SphericalPoint fromXYZ(T x, T y, T z) {
            T radius = std::sqrt(std::pow(x, 2) + std::pow(y, 2) + std::pow(z, 2));
            T theta = std::acos(z/radius);
            T phi = std::asin(y / (radius * std::sin(theta)));
            return SphericalPoint(radius, theta, phi);
        }

        static SphericalPoint fromVector(rtl::Vector3D<T> vec) {
            T radius = std::sqrt(std::pow(vec.x(), 2) + std::pow(vec.y(), 2) + std::pow(vec.z(), 2));
            T theta = std::acos(vec.z()/radius);
            T phi = std::asin(vec.y() / (radius * std::sin(theta)));
            return SphericalPoint(radius, theta, phi);
        }

        static std::vector<SphericalPoint<T>> fromPointCloud(const pcl::PointCloud<pcl::PointXYZ>& pc) {
            std::vector<SphericalPoint<T>> output;
            for (const auto& point : pc.points) {
                output.push_back(SphericalPoint<T>::fromXYZ(point.x, point.y, point.z));
            }
            return output;
        }

        rtl::Vector3D<T> toXYT() {
            T x = radius_ * std::sin(theta_) * std::cos(phi_);
            T y = radius_ * std::sin(theta_) * std::sin(phi_);
            T z = radius_ * std::cos(theta_);

            return rtl::Vector3D<T> {x, y, z};
        }

    private:
        T radius_;
        T theta_;
        T phi_;
    };
}

