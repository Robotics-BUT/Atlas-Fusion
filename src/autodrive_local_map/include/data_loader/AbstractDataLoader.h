#pragma once

#include "data_models/GenericDataModel.h"
#include "data_models/all.h"

#include <memory>
#include <vector>

namespace AutoDrive {
    namespace DataLoader {

        typedef uint64_t timestamp_type;

        enum class GnssLoaderIdentifier {
            kPose,
            kTime,
        };

        enum class ImuLoaderIdentifier {
            kDQuat,
            kGnss,
            kImu,
            kMag,
            kPressure,
            kTemp,
            kTime,
        };

        class AbstractDataLoader {

        public:

            virtual bool loadData(std::string path) = 0;
            virtual timestamp_type getLowestTimestamp() = 0;
            virtual std::shared_ptr<GenericDataModel> getNextData() = 0;
            virtual std::string toString() = 0;
            virtual uint64_t getDataSize() = 0;
            virtual bool isOnEnd() = 0;
            virtual void rewind() = 0;
            virtual void setPose(timestamp_type) = 0;
            virtual void clear() = 0;

        protected:

            std::vector<std::vector<std::string>> readCsv(const std::string&& path) const;
            std::vector<std::string> split(const std::string& s, char delimiter) const;
        };

    }
}

