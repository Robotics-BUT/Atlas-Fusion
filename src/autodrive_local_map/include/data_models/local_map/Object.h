#pragma once

#include <iostream>

namespace AutoDrive::DataModels {

    /**
     * Object represents the combination of Lidar and NN's detections combined into one. WIP
     */
    class Object {

    public:

        enum class ObjectType {
            kStaticObject,
            kDynamicObject,
        };

        enum class ObjectClass {
            kHuman,
            kBike,
            kVehicle,
            kAnimal,
            kOther,
        };

        Object() {

        }

    private:

        size_t id_;
        uint32_t ttl_;
        ObjectType objectType_;

    };
}

