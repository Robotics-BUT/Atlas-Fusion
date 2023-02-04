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

#include "local_map/LocalMap.h"

namespace AutoDrive::LocalMap {


    void LocalMap::setFrustumDetections(const std::vector<DataModels::FrustumDetection> &detections, const FrameType &sensorFrame) {
        frustumsDetections_[sensorFrame] = detections;

        // Keep track of the initial detections size and detections seen in this frame
        auto prevSize = int32_t(fusedFrustumDetections_.size());
        std::vector<bool> seenDetections(prevSize, false);

        // For every new incoming detection
        for (const auto &newDet: detections) {
            // Iterate through all existing detections
            uint32_t i = 0;
            for (const auto &det: fusedFrustumDetections_) {
                // Check if the mid-points are closer than a threshold and update the detection if so
                auto midA = newDet.getFrustum()->getNearMidPoint();
                auto midB = det.first.getFrustum()->getNearMidPoint();

                double dist = std::sqrt(std::pow(midA.x() - midB.x(), 2) + std::pow(midA.y() - midB.y(), 2) + std::pow(midA.z() - midB.z(), 2));
                if (dist < 1.5 && (newDet.getClass() == det.first.getClass())) {
                    fusedFrustumDetections_.at(i).first = interpolateBetweenFrustums(newDet, det.first);
                    fusedFrustumDetections_.at(i).second.insert(sensorFrame);
                    seenDetections.at(i) = true;
                    break;
                }
                i++;
            }

            // If the new detection hasn't been matched to anything, add it at the end of the list
            if (i == fusedFrustumDetections_.size()) {
                std::set<FrameType> set;
                set.insert(sensorFrame);
                fusedFrustumDetections_.emplace_back(newDet, set);
                seenDetections.emplace_back(true);
            }
        }

        // Iterate over all old detections in reverse and find those that hasn't been seen by this sensor
        // In that case remove this sensor from sensor list and if it ends up empty, remove the whole detection
        for (int32_t j = prevSize - 1; j >= 0; --j) {
            if(!seenDetections.at(j)) {
                auto &det = fusedFrustumDetections_.at(j);
                det.second.erase(sensorFrame);
                if (det.second.empty()) {
                    fusedFrustumDetections_.erase(fusedFrustumDetections_.begin() + j);
                }
            }
        }
    }


    void LocalMap::setLidarDetections(const std::vector<std::shared_ptr<DataModels::LidarDetection>>& detections) {
        lidarDetections_.clear();

        // For every new incoming detection
        for (const auto &newDet: detections) {
            uint32_t closest = std::numeric_limits<uint32_t>::max();

            for (const auto &det: fusedFrustumDetections_) {
                auto midA = newDet->getBoundingBox().centroid();
                auto midB = det.first.getFrustum()->getNearMidPoint();

                double dist = std::sqrt(std::pow(midA.x() - midB.x(), 2) + std::pow(midA.y() - midB.y(), 2) + std::pow(midA.z() - midB.z(), 2));

                if (dist < 1.0 && dist < closest) {
                    newDet->setDetectionClass(det.first.getClass());
                }
            }
            lidarDetections_.emplace_back(newDet);
        }
    }

    void LocalMap::setObjects(std::vector<std::shared_ptr<DataModels::Object>> objects) {
        objects_ = std::move(objects);
    }

    std::vector<std::shared_ptr<DataModels::LidarDetection>> LocalMap::getObjectsAsLidarDetections() {

        std::vector<std::shared_ptr<DataModels::LidarDetection>> output;

        return output;
    }


    std::vector<DataModels::FrustumDetection> LocalMap::getFrustumDetections() {
        std::vector<DataModels::FrustumDetection> output;
        for (auto &frustumsDetection: frustumsDetections_) {
            for (auto &frustum: frustumsDetection.second) {
                output.push_back(frustum);
            }
        }
        return output;
    }

    std::vector<std::pair<DataModels::FrustumDetection, std::set<FrameType>>> LocalMap::getFusedFrustumDetections() {
        return fusedFrustumDetections_;
    }


    std::vector<std::shared_ptr<DataModels::LidarDetection>> LocalMap::getLidarDetections() {
        return lidarDetections_;
    }


    std::vector<std::shared_ptr<DataModels::Object>> LocalMap::getObjects() {
        return objects_;
    }

    double LocalMap::getBoundingBoxVolume(const rtl::Frustum3D<double> &a) {
        return (std::abs(a.getNearBottomLeft().x() - a.getNearBottomRight().x())) * (std::abs(a.getFarBottomLeft().y() - a.getNearBottomLeft().y())) *
               (std::abs(a.getNearTopLeft().z() - a.getNearBottomLeft().z()));
    }

    double LocalMap::getBoundingBoxVolumeIntersection(const rtl::Frustum3D<double> &a,
                                                      const rtl::Frustum3D<double> &b) {
        double x_min = std::max(a.getNearBottomLeft().x(), b.getNearBottomLeft().x());
        double y_min = std::max(a.getNearBottomRight().y(), b.getNearBottomRight().y());
        double z_min = std::max(a.getNearBottomLeft().z(), b.getNearBottomLeft().z());
        double x_max = std::min(a.getFarBottomLeft().x(), b.getFarBottomLeft().x());
        double y_max = std::min(a.getNearBottomLeft().y(), b.getNearBottomLeft().y());
        double z_max = std::min(a.getNearTopLeft().z(), b.getNearTopLeft().z());

        double x_diff = std::max(x_max - x_min, 0.0);
        double y_diff = std::max(y_max - y_min, 0.0);
        double z_diff = std::max(z_max - z_min, 0.0);

        return x_diff * y_diff * z_diff;
    }

    DataModels::FrustumDetection LocalMap::interpolateBetweenFrustums(const DataModels::FrustumDetection& a, const DataModels::FrustumDetection& b) {
        auto aF = a.getFrustum();
        auto bF = b.getFrustum();

        double tX = aF->getOrigin().x() + 0.5 * (bF->getOrigin().x() - aF->getOrigin().x());
        double tY = aF->getOrigin().y() + 0.5 * (bF->getOrigin().y() - aF->getOrigin().y());
        double tZ = aF->getOrigin().z() + 0.5 * (bF->getOrigin().z() - aF->getOrigin().z());

        double ntlX = aF->getNearTopLeft().x() + 0.5 * (bF->getNearTopLeft().x() - aF->getNearTopLeft().x());
        double ntlY = aF->getNearTopLeft().y() + 0.5 * (bF->getNearTopLeft().y() - aF->getNearTopLeft().y());
        double ntlZ = aF->getNearTopLeft().z() + 0.5 * (bF->getNearTopLeft().z() - aF->getNearTopLeft().z());

        double ntrX = aF->getNearTopRight().x() + 0.5 * (bF->getNearTopRight().x() - aF->getNearTopRight().x());
        double ntrY = aF->getNearTopRight().y() + 0.5 * (bF->getNearTopRight().y() - aF->getNearTopRight().y());
        double ntrZ = aF->getNearTopRight().z() + 0.5 * (bF->getNearTopRight().z() - aF->getNearTopRight().z());

        double nblX = aF->getNearBottomLeft().x() + 0.5 * (bF->getNearBottomLeft().x() - aF->getNearBottomLeft().x());
        double nblY = aF->getNearBottomLeft().y() + 0.5 * (bF->getNearBottomLeft().y() - aF->getNearBottomLeft().y());
        double nblZ = aF->getNearBottomLeft().z() + 0.5 * (bF->getNearBottomLeft().z() - aF->getNearBottomLeft().z());

        double nbrX = aF->getNearBottomRight().x() + 0.5 * (bF->getNearBottomRight().x() - aF->getNearBottomRight().x());
        double nbrY = aF->getNearBottomRight().y() + 0.5 * (bF->getNearBottomRight().y() - aF->getNearBottomRight().y());
        double nbrZ = aF->getNearBottomRight().z() + 0.5 * (bF->getNearBottomRight().z() - aF->getNearBottomRight().z());

        auto conf = std::max(a.getDetectionConfidence(), b.getDetectionConfidence());

        rtl::Frustum3D<double> frustum({tX, tY, tZ}, {ntlX, ntlY, ntlZ}, {ntrX, ntrY, ntrZ}, {nblX, nblY, nblZ}, {nbrX, nbrY, nbrZ}, aF->getDepth());
        DataModels::FrustumDetection output(std::make_shared<rtl::Frustum3D<double>>(frustum), conf, a.getClass());
        return output;
    }
}