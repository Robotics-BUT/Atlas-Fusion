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