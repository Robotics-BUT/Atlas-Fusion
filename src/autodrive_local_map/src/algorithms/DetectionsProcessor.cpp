#include "algorithms/DetectionsProcessor.h"

namespace AutoDrive::Algorithms {

    void DetectionsProcessor::addProjector(std::shared_ptr<Projector> projector, std::string id) {
        projectors_[id] = std::move(projector);
    }


    std::vector<std::shared_ptr<DataModels::FrustumDetection>> DetectionsProcessor::onNew3DYoloDetections(std::shared_ptr<std::vector<DataModels::YoloDetection3D>> detections3D, std::string frame) {

        std::vector<std::shared_ptr<DataModels::FrustumDetection>> output{};
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
            projector->getPointDirection(points2D, points3D);

            float ratio = distance / (( points3D[0].z + points3D[1].z + points3D[2].z + points3D[3].z)/4);
            auto frustum = rtl::Frustum<double>(
                    rtl::Vector3D<double>{0,0,0},
                    rtl::Vector3D<double>{points3D[0].x * distance, points3D[0].y * distance, points3D[0].z * distance},
                    rtl::Vector3D<double>{points3D[1].x * distance, points3D[1].y * distance, points3D[1].z * distance},
                    rtl::Vector3D<double>{points3D[2].x * distance, points3D[2].y * distance, points3D[2].z * distance},
                    rtl::Vector3D<double>{points3D[3].x * distance, points3D[3].y * distance, points3D[3].z * distance},
                    1.0);

            auto tf = context_.tfTree_.getTransformationForFrame(frame);
            auto tmp = frustum.applyTransformation(tf);

            output.push_back(std::make_shared<DataModels::FrustumDetection>(
                    std::make_shared<rtl::Frustum<double>>(frustum.applyTransformation(tf)),
                    detection.getDetectionConfidence(),
                    detection.getClassConfidence(),
                    detection.getClass()));
        }
        return output;
    }
}