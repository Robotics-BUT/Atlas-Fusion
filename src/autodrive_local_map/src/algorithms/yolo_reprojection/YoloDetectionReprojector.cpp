#include "algorithms/yolo_reprojection/YoloDetectionReprojector.h"

namespace AutoDrive::Algorithms {

    std::shared_ptr<std::vector<DataModels::YoloDetection>> YoloDetectionReprojector::onNewDetections(std::vector<std::shared_ptr<const DataModels::FrustumDetection>> frustums, rtl::Transformation3D<double> currentCameraTf) {

        auto output = std::make_shared<std::vector<DataModels::YoloDetection>>();

        if(cameraProjector_ == nullptr) {
            context_.logger_.warning("Missing camera projector for projecting yolo detections");
            return output;
        }
        if(lastFrame == nullptr) {
            context_.logger_.warning("Missing camera frame for yolo detection reprojection");
            return output;
        }

        std::cout << frustums.size() << " detections" << std::endl;


        auto irImg = lastFrame->getImage();
        for(const auto& frustum : frustums) {

            const auto transformedFrustum = frustum->getFrustum()->transformed(currentCameraTf);

            auto ntl = transformedFrustum.getNearTopLeft();
            auto nbr = transformedFrustum.getNearBottomRight();

            std::vector<cv::Point3f> points3D;
            std::vector<cv::Point2f> points2D;

            points3D.emplace_back(cv::Point3f{static_cast<float>(ntl.x()), static_cast<float>(ntl.y()), static_cast<float>(ntl.z())});
            points3D.emplace_back(cv::Point3f{static_cast<float>(nbr.x()), static_cast<float>(nbr.y()), static_cast<float>(nbr.z())});

            cameraProjector_->projectPoints(points3D, points2D);

//            for(const auto& point : points2D) {
//                cv::circle(irImg, point, 5, {0});
//            }
            cv::rectangle(irImg, points2D.at(0), points2D.at(1), {255});

//            if(static_cast<size_t>(points2D.at(0).x) == 18446744073709551608) {
//                cv::imshow("bublebum", irImg);
//                cv::waitKey(0);
//            }
            output->emplace_back(DataModels::YoloDetection{static_cast<int>(points2D.at(0).x),
                                                           static_cast<int>(points2D.at(0).y),
                                                           static_cast<int>(points2D.at(1).x),
                                                           static_cast<int>(points2D.at(1).y),
                                                           frustum->getDetectionConfidence(),
                                                           frustum->getClassConfidence(),
                                                           frustum->getClass()});
        }
        return output;
    }



    void YoloDetectionReprojector::onNewIRFrame(std::shared_ptr<DataModels::CameraIrFrameDataModel> frame) {

        lastFrame = frame;
        framesCounter_++;
    }

    size_t YoloDetectionReprojector::getCurrentIrFrameNo() const {

        if(framesCounter_ == -1) {
            context_.logger_.warning("Requested IR Frame when still no frame is available");
        }

        return framesCounter_;
    }

    std::shared_ptr<DataModels::CameraIrFrameDataModel> YoloDetectionReprojector::getLastIrFrame() const {
        if(framesCounter_ == -1) {
            context_.logger_.warning("Requested IR Frame number when still no frame is available");
        }
        return lastFrame;
    }


    std::pair<int, int> YoloDetectionReprojector::getLastIrFrameWidthHeight() const {
        if(framesCounter_ == -1) {
            context_.logger_.warning("Requested IR Frame dimensions when still no frame is available");
            return {-1, -1};
        }
        return {lastFrame->getImage().cols, lastFrame->getImage().rows};
    }

}