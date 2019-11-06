#include "data_loader/data_models/imu/ImuTimeDataModel.h"

#include <sstream>

namespace AutoDrive {
    namespace DataLoader {

        std::string ImuTimeDataModel::toString() {
            std::stringstream ss;
            ss << "[Imu Time Data Model] : "
               << year_ << "." << month_ << "." << day_ << " "
               << hour_ << ":" << minute_ << ":" << sec_ << "." << nsec_;
            return ss.str();
        }
    }
}
