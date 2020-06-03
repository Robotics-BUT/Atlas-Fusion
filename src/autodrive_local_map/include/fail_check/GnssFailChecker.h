#pragma once

#include "AbstrackFailChecker.h"
#include "data_models/gnss/GnssPoseDataModel.h"
#include "data_models/gnss/GnssTimeDataModel.h"

namespace AutoDrive::FailCheck {

    /**
     * Validates GNSS receiver data packets. Currently bypassed.
     */
    class GnssFailChecker : public AbstrackFailChecker{

    public:

        GnssFailChecker() = delete;

        /**
         * Constructor
         * @param context cantainer for global services (timestamps. logging, etc.)
         */
        GnssFailChecker(Context& context)
        : AbstrackFailChecker{context}
        {

        }

        /**
         * Input for GNSS receiver global position data.
         * @param data GNSS global position data frame
         */
        void onNewData(std::shared_ptr<DataModels::GnssPoseDataModel> data);

        /**
         * Input for GNSS receiver global position data.
         * @param data GNSS global time data frame
         */
        void onNewData(std::shared_ptr<DataModels::GnssTimeDataModel> data);

    private:

    };
}

