#include "data_writers/ImageWriter.h"

namespace AtlasFusion::DataWriters {

    void ImageWriter::saveImage(cv::Mat img,
                                size_t frameNo,
                                const std::string& subdir,
                                const std::string& sufix,
                                const std::string& fileExtention) {

        std::stringstream pathStream;
        pathStream << destFolder_ << subdir << "frame" << std::setw(6) << std::setfill('0') << frameNo << sufix << "." << fileExtention;
        std::cout << pathStream.str() << std::endl;
        cv::imwrite(pathStream.str(), img);
    }
}