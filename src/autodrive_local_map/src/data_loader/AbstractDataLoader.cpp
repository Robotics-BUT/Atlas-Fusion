#include "data_loader/AbstractDataLoader.h"

#include <fstream>
#include <sstream>

namespace AutoDrive {
    namespace DataLoader {


        std::vector<std::vector<std::string>> AbstractDataLoader::readCsv(const std::string&& path) const {
            std::vector<std::vector<std::string>> output;

            std::ifstream file;
            file.open(path);
            if (file.is_open()) {
                std::string line;
                while (std::getline(file, line)) {
                    output.push_back(split(line, ','));
                }
            }
            else {
                std::cerr << "Unable to open data file: " << path << std::endl;
            }

            return output;
        }


        std::vector<std::string> AbstractDataLoader::split(const std::string& s, char delimiter) const{
            std::vector<std::string> tokens;
            std::string token;
            std::istringstream tokenStream(s);
            while (std::getline(tokenStream, token, delimiter))
            {
                tokens.push_back(token);
            }
            return tokens;
        }
    }
}