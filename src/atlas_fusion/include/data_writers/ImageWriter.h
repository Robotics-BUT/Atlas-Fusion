#pragma once

#include <filesystem>

#include <opencv2/opencv.hpp>
#include "Context.h"
#include "data_loader/RecordingConstants.h"

namespace AtlasFusion::DataWriters {

    class ImageWriter {
    public:

        ImageWriter(Context& context, std::string dstFold)
                : context_{context}
                , destFolder_{std::move(dstFold)} {

            destFolder_ += DataLoader::Folders::kOutputFolder;
            if( !std::filesystem::exists(destFolder_) ) {
                std::filesystem::create_directory(destFolder_);
            }
        }

        /**
         * Method stores image into the file system
         * @param img image to be stored
         * @param frameNo frame number that defines on-disk image name
         */
        void saveImage(cv::Mat img, size_t frameNo, const std::string& subdir, const std::string& sufix, const std::string& fileExtention);

    private:
        Context& context_;
        std::string destFolder_;
    };
}
