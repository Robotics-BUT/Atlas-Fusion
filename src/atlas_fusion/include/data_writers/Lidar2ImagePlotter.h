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

#pragma once

#include <opencv2/opencv.hpp>
#include <filesystem>

#include "Context.h"
#include "data_loader/RecordingConstants.h"

namespace AtlasFusion::DataWriters {

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

            destFolder_ += DataLoader::Folders::kOutputFolder;
            if( !std::filesystem::exists(destFolder_) ) {
                std::filesystem::create_directory(destFolder_);
            }

            destFolder_ += DataLoader::Folders::kOutputDepthMap;
            if( !std::filesystem::exists(destFolder_) ) {
                std::filesystem::create_directory(destFolder_);
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
        std::shared_ptr<cv::Mat> renderLidarPointsToImg(std::vector<cv::Point2f>, std::vector<cv::Point3f>, int imgWidth, int imgHeightm, size_t pointSize=1);



    private:

        Context& context_;
        float maxDist_;
        std::string destFolder_;

        uint8_t distanceToColor(float dist);
        float pointLenght(cv::Point3f&);


    };

}

