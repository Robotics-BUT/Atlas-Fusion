#pragma once

#include <iostream>

namespace AutoDrive::DataModels {

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

