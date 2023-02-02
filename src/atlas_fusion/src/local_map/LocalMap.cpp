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


    void LocalMap::setFrustumDetections(const std::vector<DataModels::FrustumDetection> &detections,
                                        const FrameType &sensorFrame) {
        frustumsDetections_[sensorFrame] = detections;

        for (auto newDetection: detections) {
            if(fusedFrustumDetections_.empty()) {
                fusedFrustumDetections_[newDetection] = {};
            }
            for (auto &prevDetection: fusedFrustumDetections_) {
                const auto &a = newDetection.getFrustum().;
                const auto &b = prevDetection.first.getFrustum();

                double intersection = getFrustumVolumeIntersection(a, b);
                if(intersection > 0.9)
                auto d = 1;
            }
        }
    }


    void LocalMap::setLidarDetections(std::vector<std::shared_ptr<DataModels::LidarDetection>> detections) {
        lidarDetections_ = std::move(detections);
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


    std::vector<std::shared_ptr<DataModels::LidarDetection>> LocalMap::getLidarDetections() {
        return lidarDetections_;
    }


    std::vector<std::shared_ptr<DataModels::Object>> LocalMap::getObjects() {
        return objects_;
    }


    float LocalMap::getFrustumVolumeIntersection(const std::shared_ptr<rtl::Frustum3D<double> const> &a,
                                                 const std::shared_ptr<rtl::Frustum3D<double> const> &b) {
        double x_min = std::max(a->getNearBottomLeft().x(), b->getNearBottomLeft().x());
        double y_min = std::max(a->getNearBottomRight().y(), b->getNearBottomRight().y());
        double z_min = std::max(a->getNearBottomLeft().z(), b->getNearBottomLeft().z());
        double x_max = std::min(a->getFarBottomLeft().x(), b->getFarBottomLeft().x());
        double y_max = std::min(a->getNearBottomLeft().y(), b->getNearBottomLeft().y());
        double z_max = std::min(a->getNearTopLeft().z(), b->getNearTopLeft().z());

        double x_diff = std::max(x_max - x_min, 0.0);
        double y_diff = std::max(y_max - y_min, 0.0);
        double z_diff = std::max(z_max - z_min, 0.0);

        return x_diff * y_diff * z_diff;
    }
}