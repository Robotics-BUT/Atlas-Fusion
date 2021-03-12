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

#include "LogService.h"

namespace AtlasFusion {


    bool LogService::openFile() {
        try {
            file_.open(logPath_);
        } catch (std::exception& e ) {
            std::cerr << e.what() << std::endl;
        }
         return file_.is_open();
    }


    void LogService::debug(const std::string& msg) {
        if (logLvl_ <= LogLevel::Debug) {
            if(toFile_ && file_.is_open()) {
                file_ << msg << std::endl;
            }
            if(toConsole_) {
                std::cout << msg << std::endl;
            }
        }
    }


    void LogService::info(const std::string& msg) {
        if (logLvl_ <= LogLevel::Info) {
            if(toFile_ && file_.is_open()) {
                file_ << msg << std::endl;
            }
            if(toConsole_) {
                std::cout << msg << std::endl;
            }
        }
    }


    void LogService::warning(const std::string& msg) {
        if (logLvl_ <= LogLevel::Warning) {
            if(toFile_ && file_.is_open()) {
                file_ << msg << std::endl;
            }
            if(toConsole_) {
                std::cout << msg << std::endl;
            }
        }
    }


    void LogService::error(const std::string& msg) {
        if (logLvl_ <= LogLevel::Error) {
            if(toFile_ && file_.is_open()) {
                file_ << msg << std::endl;
            }
            if(toConsole_) {
                std::cout << msg << std::endl;
            }
        }
    }

    const std::string LogService::currentDateTime() {
        time_t     now = time(0);
        struct tm  tstruct;
        char       buf[80];
        tstruct = *localtime(&now);
        strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);

        return buf;
    }
}