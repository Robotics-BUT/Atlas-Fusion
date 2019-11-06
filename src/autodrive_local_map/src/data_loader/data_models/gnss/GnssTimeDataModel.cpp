#include "data_loader/data_models/gnss/GnssTimeDataModel.h"

#include <sstream>

namespace AutoDrive {
    namespace DataLoader {

        std::string GnssTimeDataModel::toString() {
            std::stringstream ss;
            ss << "[Gnss Time Data Model] : "
               << year_ << "." << month_ << "." << day_ << " "
               << hour_ << ":" << minute_ << ":" << sec_ << "." << nsec_;
            return ss.str();
        }
    }
}
