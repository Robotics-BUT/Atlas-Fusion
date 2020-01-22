# pragma once

#include "Context.h"
#include "data_models/GenericDataModel.h"

namespace AutoDrive::FailCheck {

    class AbstrackFailChecker {


    public:

        AbstrackFailChecker() = delete;

        explicit AbstrackFailChecker(Context& context) :
        context_{context}
        {
            sensorStatus_ = 1.0;
        }

        virtual float getSensorStatus();

    protected:

        Context& context_;
        float sensorStatus_;

    };
}
