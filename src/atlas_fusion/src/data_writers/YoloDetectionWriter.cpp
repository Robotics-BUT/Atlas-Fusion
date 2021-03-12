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

#include "data_writers/YoloDetectionWriter.h"

#include <opencv2/opencv.hpp>
#include <data_loader/RecordingConstants.h>

namespace AtlasFusion::DataWriters {

    void YoloDetectionWriter::writeDetections(std::shared_ptr<std::vector<DataModels::YoloDetection>> detections, size_t frameNo) {

        if(!outputFile_.is_open()) {
            auto file_path = destinationDir_ + destinationFile_;
            openFile(file_path);
            if(!outputFile_.is_open()) {
                context_.logger_.warning("Unable to open " + file_path + " file");
                return;
            }
        }
        
        for (const auto& detection : *detections) {
            outputFile_ << frameNo << ", "
                        << detection.getBoundingBox().x1_ << ", "
                        << detection.getBoundingBox().y1_ << ", "
                        << detection.getBoundingBox().x2_ << ", "
                        << detection.getBoundingBox().y2_ << ", "
                        << detection.getDetectionConfidence() << ", "
                        << static_cast<int>(detection.getDetectionClass()) << std::endl;
        }
    }


    void YoloDetectionWriter::writeDetectionsAsTrainData(std::shared_ptr<std::vector<DataModels::YoloDetection>> detections, size_t frameNo, int image_width, int image_height) {

        std::ofstream outputDetFile;
        std::stringstream ss;
        ss << std::setw(10) << std::setfill('0');
        ss << frameNo << ".txt";

        auto file_path = destinationDir_ + ss.str();
        outputDetFile.open(file_path);

        if (!outputDetFile.is_open()) {
            context_.logger_.warning("Unable to open file for ir yolo training labels");
            return;
        }
        for (const auto &detection : *detections) {

            int x1 = std::min(std::max(detection.getBoundingBox().x1_, 0), image_width-2);
            int y1 = std::min(std::max(detection.getBoundingBox().y1_, 0), image_width-2);
            int x2 = std::min(std::max(detection.getBoundingBox().x2_, x1+1), image_width-1);
            int y2 = std::min(std::max(detection.getBoundingBox().y2_, y1+1), image_width-1);

            float xCenter = (x1 + x2) / 2.0f;
            float yCenter = (y1 + y2) / 2.0f;
            float width = (x2 - x1);
            float height = (y2 - y1);

            outputDetFile << static_cast<size_t>(detection.getDetectionClass()) << " "
                          << xCenter / image_width << " "
                          << yCenter / image_height << " "
                          << width * 1.2 / image_width << " "
                          << height * 1.4/ image_width << std::endl;
        }
        outputDetFile.close();
    }


    void YoloDetectionWriter::writeIRImageAsTrainData(std::shared_ptr<DataModels::CameraIrFrameDataModel> frame, size_t frameNo) {

        std::stringstream ss;
        ss << "frame" << std::setw(6) << std::setfill('0') << frameNo << ".jpeg";
        auto file_path = destinationDir_ + ss.str();

        cv::imwrite(file_path, frame->getImage());
    }


    void YoloDetectionWriter::changeDestinationFile(std::string destinationDir, std::string destinationFile) {

        openFile(destinationDir + destinationFile);
    }


    void YoloDetectionWriter::openFile(std::string path) {

        if(outputFile_.is_open()) {
            outputFile_.close();
        }

        outputFile_.open(path,  std::ios::out);
    }
}