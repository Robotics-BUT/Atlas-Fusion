#include "algorithms/pointcloud/LaserSegmenter.h"

namespace AutoDrive::Algorithms {


    void LaserSegmenter::onLaserData(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> laserData, size_t laserNo) {

        if(laserNo >= segmenters_.size()) {
            context_.logger_.warning("Laser index out of range in LaserSegmenter::onLaserData method!");
            return;
        }

        auto& segmenter = segmenters_.at(laserNo);

        for (const auto p : laserData->points) {

            if (std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.y))
                continue;

            if (p.x == 0 && p.y == 0 && p.z == 0)
                continue;


            auto point = rtl::Vector3D<double>{p.x, p.y, p.z};
            auto origin = rtl::Vector3D<double>{0.0, 0.0, 0.0};

            auto zDirection = rtl::Vector3D<double>::baseZ();
            segmenter.addPoint(point, origin);

            while (segmenter.closedClustersAvailable() != 0) {
                auto seg = segmenter.grabCluster();
                if (seg.size() > 100) {
                    if (vectorizer_(seg)) {
                        auto ls = vectorizer_.lineSegments();
                        for (const auto &l : ls) {
                            if (std::abs(l.direction().dot(zDirection)) < 0.2 &&
                                std::abs(rtl::Vector3D<double>::scalarProjectionOnUnit(
                                        rtl::Vector3D<double>{l.beg().x(), l.beg().y(), l.beg().z()} -
                                        origin,
                                        zDirection) + 1.5) < 0.3 &&
                                std::abs(rtl::Vector3D<double>::scalarProjectionOnUnit(
                                        rtl::Vector3D<double>{l.end().x(), l.end().y(), l.end().z()} -
                                        origin,
                                        zDirection) + 1.5) < 0.3)
                                lineSegmentsRoad_[laserNo].emplace_back(l);
                            else {
                                lineSegments_[laserNo].emplace_back(l);
                            }
                        }
                    }
                }
            }
        }
    }


    std::shared_ptr<std::vector<rtl::LineSegment3D<double>>> LaserSegmenter::getApproximation(size_t laserNo) {
        auto output = std::make_shared<std::vector<rtl::LineSegment3D<double>>>();
        for (auto &l : lineSegments_[laserNo])
            output->push_back(l);
        return output;
    }


    std::shared_ptr<std::vector<rtl::LineSegment3D<double>>> LaserSegmenter::getAllApproximations() {
        auto output = std::make_shared<std::vector<rtl::LineSegment3D<double>>>();
        for (const auto & ls : lineSegments_)
            for (auto &l : ls)
                output->push_back(l);
        return output;
    }


    std::shared_ptr<std::vector<rtl::LineSegment3D<double>>> LaserSegmenter::getRoad(size_t laserNo) {
        auto output = std::make_shared<std::vector<rtl::LineSegment3D<double>>>();
        for (auto &l : lineSegmentsRoad_[laserNo])
            output->push_back(l);
        return output;
    }


    std::shared_ptr<std::vector<rtl::LineSegment3D<double>>> LaserSegmenter::getAllRoads() {
        auto output = std::make_shared<std::vector<rtl::LineSegment3D<double>>>();
        for (const auto & ls : lineSegmentsRoad_)
            for (auto &l : ls)
                output->push_back(l);
        return output;
    }


    void LaserSegmenter::clear() {
        lineSegments_.clear();
        lineSegments_.resize(noOfLasers_);
        lineSegmentsRoad_.clear();
        lineSegmentsRoad_.resize(noOfLasers_);
    }
}