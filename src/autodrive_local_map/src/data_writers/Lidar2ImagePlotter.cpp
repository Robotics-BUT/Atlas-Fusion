#include "data_writers/Lidar2ImagePlotter.h"
#include <algorithm>

#include "data_loader/RecordingConstants.h"

namespace AutoDrive::DataWriters {


    std::shared_ptr<cv::Mat> Lidar2ImagePlotter::renderLidarPointsToImg(std::vector<cv::Point2f> points2D, std::vector<cv::Point3f> points3D, int imgWidth, int imgHeight) {

        auto image = std::make_shared<cv::Mat>(imgHeight, imgWidth, CV_8U, cv::Scalar(0));

        if(points2D.size() != points3D.size()) {
            context_.logger_.warning("points2D vector size is not equal to points3D vector size in Lidar2Image!");
        }

        for(size_t i = 0 ; i < points2D.size() ; i++) {
            image->at<uint8_t >(static_cast<int>(points2D.at(i).y), static_cast<int>(points2D.at(i).x)) = distanceToColor( pointLenght(points3D.at(i)) );
        }

        return image;
    }


    void Lidar2ImagePlotter::saveImage(std::shared_ptr<cv::Mat> img, size_t frameNo) {

        std::stringstream pathStream;

        pathStream << std::setw(6) << std::setfill('0');
        pathStream << destFolder_ + DataLoader::Folders::kLidarDepth << "frame" << frameNo << ".jpeg";

        cv::imwrite(pathStream.str(), *img);
    }


    uint8_t Lidar2ImagePlotter::distanceToColor(float dist) {

        auto val = (-255.0/maxDist_) * dist + 255.0;
        return static_cast<uint8_t> (std::max(std::min(val, 255.0), 0.0));
    }


    float Lidar2ImagePlotter::pointLenght(cv::Point3f& p) {
        return std::sqrt( p.x * p.x  +  p.y * p.y  +  p.z * p.z);
    }
}