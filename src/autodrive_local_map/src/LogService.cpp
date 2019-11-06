#include "LogService.h"

namespace AutoDrive {


    bool LogService::openFile() {
        try {
            file_.open(logPath_);
        } catch (std::exception e ) {
            std::cerr << e.what() << std::endl;
        }
         return file_.is_open();
    }


    void LogService::debug(const std::string& msg) {
        if (logLvl_ >= LogLevel::Debug) {
            if(toFile_ && file_.is_open()) {
                file_ << msg << std::endl;
            }
            if(toConsole_) {
                std::cout << msg << std::endl;
            }
        }
    }


    void LogService::info(const std::string& msg) {
        if (logLvl_ >= LogLevel::Info) {
            if(toFile_ && file_.is_open()) {
                file_ << msg << std::endl;
            }
            if(toConsole_) {
                std::cout << msg << std::endl;
            }
        }
    }


    void LogService::warning(const std::string& msg) {
        if (logLvl_ >= LogLevel::Warning) {
            if(toFile_ && file_.is_open()) {
                file_ << msg << std::endl;
            }
            if(toConsole_) {
                std::cout << msg << std::endl;
            }
        }
    }


    void LogService::error(const std::string& msg) {
        if (logLvl_ >= LogLevel::Error) {
            if(toFile_ && file_.is_open()) {
                file_ << msg << std::endl;
            }
            if(toConsole_) {
                std::cout << msg << std::endl;
            }
        }
    }
}