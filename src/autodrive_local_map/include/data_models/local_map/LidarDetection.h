#pragma once

#include <rtl/BoundingBox.h>

namespace AutoDrive::DataModels {

    class LidarDetection {

    public:

        LidarDetection() = delete;
        LidarDetection(rtl::BoundingBox3d box, size_t id, uint32_t ttl = 10)
        : box_{std::move(box)}
        , id_{id}
        , ttl_{ttl} {

        }

        rtl::BoundingBox3d getBoundingBox() const { return box_; }
        size_t getID() const { return id_; }
        size_t getTTL() const { return ttl_; }

    private:

        rtl::BoundingBox3d box_;
        size_t id_;
        uint32_t ttl_;
    };
}

