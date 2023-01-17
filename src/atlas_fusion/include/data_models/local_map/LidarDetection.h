/*
 * Copyright 2020 Brno University of Technology
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom
 * the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT
 * OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
 * OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#pragma once

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
        LidarDetection(const rtl::BoundingBox3d &box, const rtl::Quaterniond& quat, size_t id, uint32_t ttl = 10)
                : box_{box}, quat_{quat}, id_{id}, ttl_{ttl} {}

        /**
         * Axis-Aligned Bounding box getter
         * @return bounding box that defines obstacle dimensions
         */
        [[nodiscard]] rtl::BoundingBox3d getBoundingBox() const { return box_; }

        /**
        * Rotation getter
        * @return quaternion that defines obstacle rotation
        */
        [[nodiscard]] rtl::Quaterniond getRotation() const { return quat_; }

        /**
         * Obstacle's ID getter
         * @return ID
         */
        [[nodiscard]] size_t getID() const { return id_; }

        /**
         * Obstacle's Time to Live
         * @return remaining time to live
         */
        [[nodiscard]] size_t getTTL() const { return ttl_; };

    private:

        rtl::BoundingBox3d box_;
        rtl::Quaterniond quat_;
        size_t id_;
        uint32_t ttl_;
    };
}

