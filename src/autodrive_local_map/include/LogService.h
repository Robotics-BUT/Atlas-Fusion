#pragma once

#include <iostream>
#include <string>
#include <fstream>

namespace AutoDrive {
    class LogService {


    public:

        enum class LogLevel {
            Debug = 0,
            Info = 1,
            Warning = 2,
            Error = 3,
        };

        LogService(std::string &&path, LogLevel& lvl, bool console, bool file)
        : logPath_(path)
        , logLvl_(lvl)
        , toConsole_(console)
        , toFile_(file) {
        }

        LogService(const LogService&) = delete;

        bool openFile();

        void debug(const std::string& msg);
        void info(const std::string& msg);
        void warning(const std::string& msg);
        void error(const std::string& msg);
        static const std::string currentDateTime();

    private:

        std::string logPath_;
        LogLevel logLvl_;
        bool toConsole_;
        bool toFile_;
        std::ofstream file_;
    };
}

