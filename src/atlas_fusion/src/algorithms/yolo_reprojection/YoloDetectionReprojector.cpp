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

#include "algorithms/yolo_reprojection/YoloDetectionReprojector.h"

namespace AtlasFusion::Algorithms {

    std::shared_ptr<std::vector<DataModels::YoloDetection>> YoloDetectionReprojector::onNewDetections(std::vector<std::shared_ptr<const DataModels::FrustumDetection>> frustums, rtl::RigidTf3D<double> currentCameraTf) {

        auto output = std::make_shared<std::vector<DataModels::YoloDetection>>();

        if(cameraProjector_ == nullptr) {
            context_.logger_.warning("Missing camera projector for projecting yolo detections");
            return output;
        }

        std::cout << frustums.size() << " detections" << std::endl;

        for(const auto& frustum : frustums) {

            const auto transformedFrustum = frustum->getFrustum()->transformed(currentCameraTf);

            auto ntl = transformedFrustum.getNearTopLeft();
            auto nbr = transformedFrustum.getNearBottomRight();

            std::vector<cv::Point3f> points3D;
            std::vector<cv::Point2f> points2D;

            points3D.emplace_back(cv::Point3f{static_cast<float>(ntl.x()), static_cast<float>(ntl.y()), static_cast<float>(ntl.z())});
            points3D.emplace_back(cv::Point3f{static_cast<float>(nbr.x()), static_cast<float>(nbr.y()), static_cast<float>(nbr.z())});

            cameraProjector_->projectPoints(points3D, points2D);
            output->emplace_back(DataModels::YoloDetection{static_cast<int>(points2D.at(0).x),
                                                           static_cast<int>(points2D.at(0).y),
                                                           static_cast<int>(points2D.at(1).x),
                                                           static_cast<int>(points2D.at(1).y),
                                                           frustum->getDetectionConfidence(),
                                                           frustum->getClass()});
        }
        return output;
    }
}