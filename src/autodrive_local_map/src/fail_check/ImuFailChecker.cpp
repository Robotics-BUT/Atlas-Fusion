#include "fail_check/ImuFailChecker.h"

namespace AutoDrive::FailCheck {


    void ImuFailChecker::onNewData(std::shared_ptr<DataModels::ImuDquatDataModel> /*data*/) {

    }

    void ImuFailChecker::onNewData(std::shared_ptr<DataModels::ImuImuDataModel> /*data*/) {

    }

    void ImuFailChecker::onNewData(std::shared_ptr<DataModels::ImuGnssDataModel> /*data*/) {

    }

    void ImuFailChecker::onNewData(std::shared_ptr<DataModels::ImuMagDataModel> /*data*/) {

    }

    void ImuFailChecker::onNewData(std::shared_ptr<DataModels::ImuPressureDataModel> /*data*/) {

    }

    void ImuFailChecker::onNewData(std::shared_ptr<DataModels::ImuTempDataModel> /*data*/) {

    }

    void ImuFailChecker::onNewData(std::shared_ptr<DataModels::ImuTimeDataModel> /*data*/) {

    }

}
