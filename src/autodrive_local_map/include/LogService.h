#pragma once

#include <iostream>
#include <string>
#include <fstream>

namespace AutoDrive {

    /**
     *  Log Service, the class that is used as a global logger.
     */

    class LogService {

    public:

        enum class LogLevel {
            Debug = 0,
            Info = 1,
            Warning = 2,
            Error = 3,
            Off = 4,
        };

        /**
         * Constructor
         * @param path system path, where the log file will be generated
         * @param lvl the minimal level of messages that will be logger
         * @param console if true, Log Service will be logging into the console
         * @param console if true, Log Service will be logging into the file
         */
        LogService(std::string &&path, LogLevel lvl, bool console, bool file)
        : logPath_(path)
        , logLvl_(lvl)
        , toConsole_(console)
        , toFile_(file) {
        }

        LogService(const LogService&) = delete;

        /**
         * Method explicitly opens log file.
         * @return true if file has been opened succesfuly
         */
        bool openFile();

        /**
         * Log message on the Debug level and higher
         * @param msg message to log
         */
        void debug(const std::string& msg);

        /**
         * Log message on the Info level and higher
         * @param msg message to log
         */
        void info(const std::string& msg);

        /**
         * Log message on the Warning level and higher
         * @param msg message to log
         */
        void warning(const std::string& msg);

        /**
         * Log message on the Error level and higher
         * @param msg message to log
         */
        void error(const std::string& msg);

        /**
         * Returns current date as a formated string
         * @param formated date
         */
        static const std::string currentDateTime();

    private:

        std::string logPath_;
        LogLevel logLvl_;
        bool toConsole_;
        bool toFile_;
        std::ofstream file_;
    };
}

