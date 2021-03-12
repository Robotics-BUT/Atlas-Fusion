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

#include <iostream>
#include <fstream>
#include <filesystem>

#include "Context.h"

#include "data_models/yolo/YoloDetection.h"
#include "data_models/camera/CameraIrFrameDataModel.h"
#include "data_loader/RecordingConstants.h"

namespace AtlasFusion::DataWriters {

    /**
     * Yolo Detection Writer is used to store reprojected neural networks detections from one camera to another on the
     * local disk storage
     */
    class YoloDetectionWriter {

    public:

        YoloDetectionWriter() = delete;

        /**
         * Constructor
         * @param context global services container (timestamp, logging, etc.)
         * @param destinationDir folder in which the reprojected detection will be stored
         * @param destinationFile the file name into which the writer will store the data
         */
        YoloDetectionWriter(Context& context, std::string destinationDir, std::string destinationFile)
        : context_{context}
        , destinationDir_{std::move(destinationDir)}
        , destinationFile_{std::move(destinationFile)} {

            destinationDir_ += DataLoader::Folders::kOutputFolder;
            if( !std::filesystem::exists(destinationDir_) ) {
                std::filesystem::create_directory(destinationDir_);
            }

            destinationDir_ = destinationDir_ + DataLoader::Folders::kYoloFolder;
            if( !std::filesystem::exists(destinationDir_) ) {
                std::filesystem::create_directory(destinationDir_);
            }
        }

        /**
         * Method writes given detections into the file
         * @param detections vector of the detection to be writen
         * @param frameNo frame number that detections corresponds to
         */
        void writeDetections(std::shared_ptr<std::vector<DataModels::YoloDetection>> detections, size_t frameNo);

        /**
         * Method writes data in the format COCO dataset, so it is compatible with YOLO input
         * @param detections detections to be writen
         * @param frameNo frame number that detections corresponds to
         * @param image_width image width
         * @param image_height image height
         */
        void writeDetectionsAsTrainData(std::shared_ptr<std::vector<DataModels::YoloDetection>> detections, size_t frameNo, int image_width, int image_height);

        /**
         * Method stores IR image on the dist in the format that is compatible with YOLO training data
         * @param frame IR image frame
         * @param frameNo IR image frame number
         */
        void writeIRImageAsTrainData(std::shared_ptr<DataModels::CameraIrFrameDataModel> frame, size_t frameNo);

        /**
         * Changes location where the data should be stored
         * @param destinationDir storage folder
         * @param destinationFile storage file name
         */
        void changeDestinationFile(std::string destinationDir, std::string destinationFile);

    private:

        Context& context_;
        std::string destinationDir_;
        std::string destinationFile_;

        std::ofstream outputFile_;

        void openFile(std::string path);
    };

}