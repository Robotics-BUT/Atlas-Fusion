# pragma once

#include "Context.h"
#include "data_models/GenericDataModel.h"

namespace AutoDrive::FailCheck {

    /**
     * Abstract Fail Checker defines interface for other inherited Fail Checkers
     */
    class AbstrackFailChecker {

    public:

        AbstrackFailChecker() = delete;

        /**
         * Constructor
         * @param context cantainer for global services (timestamps. logging, etc.)
         */
        explicit AbstrackFailChecker(Context& context) :
        context_{context}
        {
            sensorStatus_ = 1.0;
        }

        /**
         * Reports reliability of the sensor
         * @return
         */
        virtual float getSensorStatus();

    protected:

        Context& context_;
        float sensorStatus_;

    };
}
