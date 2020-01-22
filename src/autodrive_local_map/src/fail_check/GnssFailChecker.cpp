#include "fail_check/GnssFailChecker.h"

namespace AutoDrive::FailCheck {


    void GnssFailChecker::onNewData(std::shared_ptr<DataModels::GnssPoseDataModel> /*data*/) {
        return;
    }


    void GnssFailChecker::onNewData(std::shared_ptr<DataModels::GnssTimeDataModel> /*data*/) {
        return;
    }
}