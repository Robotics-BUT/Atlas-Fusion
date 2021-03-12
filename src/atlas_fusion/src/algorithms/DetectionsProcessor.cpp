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

#include "algorithms/DetectionsProcessor.h"

namespace AtlasFusion::Algorithms {

    void DetectionsProcessor::addProjector(std::shared_ptr<Projector> projector, std::string id) {
        projectors_[id] = std::move(projector);
    }


    std::vector<std::shared_ptr<const DataModels::FrustumDetection>> DetectionsProcessor::onNew3DYoloDetections(std::shared_ptr<std::vector<DataModels::YoloDetection3D>> detections3D, std::string frame) {

        std::vector<std::shared_ptr<const DataModels::FrustumDetection>> output{};
        auto projector = projectors_[frame];
        for(const auto& detection : *detections3D) {
            auto boundingBox = detection.getBoundingBox();
            auto distance = detection.getDistance();

            std::vector<cv::Point2f> points2D{
                    cv::Point2f{ boundingBox.x1_, boundingBox.y1_ },
                    cv::Point2f{ boundingBox.x2_, boundingBox.y1_ },
                    cv::Point2f{ boundingBox.x1_, boundingBox.y2_ },
                    cv::Point2f{ boundingBox.x2_, boundingBox.y2_ },
            };

            std::vector<cv::Point3f> points3D;
            projector->reverseProjection(points2D, points3D, false);

            auto frustum = rtl::Frustum3D<double>(
                    rtl::Vector3D<double>{0,0,0},
                    rtl::Vector3D<double>{points3D[0].x * distance, points3D[0].y * distance, points3D[0].z * distance},
                    rtl::Vector3D<double>{points3D[1].x * distance, points3D[1].y * distance, points3D[1].z * distance},
                    rtl::Vector3D<double>{points3D[2].x * distance, points3D[2].y * distance, points3D[2].z * distance},
                    rtl::Vector3D<double>{points3D[3].x * distance, points3D[3].y * distance, points3D[3].z * distance},
                    1.0);

            auto tf = context_.tfTree_.getTransformationForFrame(frame);

            output.push_back(std::make_shared<const DataModels::FrustumDetection>(
                    std::make_shared<rtl::Frustum3D<double>>(frustum.transformed(tf)),
                    detection.getDetectionConfidence(),
                    detection.getClass()));
        }
        return output;
    }
}