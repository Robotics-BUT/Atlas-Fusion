#pragma once

#include "AbstrackFailChecker.h"
#include "data_models/gnss/GnssPoseDataModel.h"
#include "data_models/gnss/GnssTimeDataModel.h"

namespace AutoDrive::FailCheck {

    class GnssFailChecker : public AbstrackFailChecker{

    public:

        GnssFailChecker() = delete;

        GnssFailChecker(Context& context)
        : AbstrackFailChecker{context}
        {

        }

        void onNewData(std::shared_ptr<DataModels::GnssPoseDataModel>);
        void onNewData(std::shared_ptr<DataModels::GnssTimeDataModel>);

    private:

    };
}

