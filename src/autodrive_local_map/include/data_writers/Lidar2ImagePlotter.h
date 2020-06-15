#pragma once

#include <opencv2/opencv.hpp>
#include <filesystem>

#include "Context.h"
#include "data_loader/RecordingConstants.h"

namespace AutoDrive::DataWriters {

    /**
     * Lidar to Image Plotter handler the simple 2D rendering of the given points into the image matrix and it is able
     * store the rendered image in the file system
     */
    class Lidar2ImagePlotter {

    public:


        Lidar2ImagePlotter() = delete;

        /**
         * Constructor
         * @param context global services container (time, TF tree, logging service)
         * @param maxDistance max distance of the point that will be rendered into the image
         * @param destFolder folder in which the rendered images will be stored
         */
        Lidar2ImagePlotter(Context& context, float maxDistance, std::string destFolder)
        : context_{context}
        , maxDist_{maxDistance}
        , destFolder_{std::move(destFolder)} {

            auto directory = destFolder_ + DataLoader::Folders::kLidarDepth;
            if( !std::filesystem::exists(directory) ) {
                std::filesystem::create_directory(directory);
            }
        }

        /**
         * Method renders depth map by drawing given points into the image plane.
         * @param points2D image coordinates (u, v) of the rendered points
         * @param points3D 3D space coordinates (x, y, z) of the rendered points (z is used)
         * @param imgWidth rendered image widht
         * @param imgHeight rendered image height
         * @return rendered depth image
         */
        std::shared_ptr<cv::Mat> renderLidarPointsToImg(std::vector<cv::Point2f>, std::vector<cv::Point3f>, int imgWidth, int imgHeight);

        /**
         * Method stores image into the file system
         * @param img image to be stored
         * @param frameNo frame number that defines on-disk image name
         */
        void saveImage(std::shared_ptr<cv::Mat> img, size_t frameNo);

    private:

        Context& context_;
        float maxDist_;
        std::string destFolder_;

        uint8_t distanceToColor(float dist);
        float pointLenght(cv::Point3f&);


    };

}

