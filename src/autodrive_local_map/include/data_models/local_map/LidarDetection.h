#pragma once

#include <rtl/Core.h>

namespace AutoDrive::DataModels {

    /**
     * Lidar Detection represents obstacle detected in the point cloud by the 3D bounding box
     */
    class LidarDetection {

    public:

        LidarDetection() = delete;

        /**
         * Constructor
         * @param box 3D bounding box that defines obstacle dimensions
         * @param id obstacle's  ID
         * @param ttl obstacle's time to live
         */
        LidarDetection(rtl::BoundingBox3d box, size_t id, uint32_t ttl = 10)
        : box_{std::move(box)}
        , id_{id}
        , ttl_{ttl} {

        }

        /**
         * Bounding box getter
         * @return bounding box that defines obstacle dimensions
         */
        rtl::BoundingBox3d getBoundingBox() const { return box_; }

        /**
         * Obstacle's ID getter
         * @return ID
         */
        size_t getID() const { return id_; }

        /**
         * Obstacle's Time to Live
         * @return remining time to live
         */
        size_t getTTL() const { return ttl_; }

    private:

        rtl::BoundingBox3d box_;
        size_t id_;
        uint32_t ttl_;
    };
}

