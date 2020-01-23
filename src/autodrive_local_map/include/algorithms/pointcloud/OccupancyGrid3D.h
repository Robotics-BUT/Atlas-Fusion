#pragma once

#include "Context.h"

namespace AutoDrive::Algorithms {

    class OccupancyGrid3D {

    public:

        explicit OccupancyGrid3D(Context& context)
        : context_{context} {

        }

    private:

        Context& context_;

    };
}
