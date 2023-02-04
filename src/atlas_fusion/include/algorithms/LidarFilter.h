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

#include "data_models/lidar/LidarScanDataModel.h"
#include "util/IdentifierToFrameConversions.h"

namespace AutoDrive::Algorithms {

    /**
     * Lidar Filter class encapsulates simple tools that allows to filter input point cloud data that comes directly
     * from the lidar scanner
     */
    class LidarFilter {

    public:

        /**
         * Method calls the set of filtering actions on the incoming lidar scan
         * @param data lidar scan
         */
        void applyFiltersOnLidarData(pcl::PointCloud<pcl::PointXYZ>::Ptr &data) {
            Timer t("Lidar filter");
            if (filterNearObjects_) {
                filterNearObjects(data);
            }
            if (filterOutliers_) {
                //filterOutliers(data);
            }
        }

        /**
         * Set the flag that enables point cloud filtration
         */
        void enableFilterNearObjects() { filterNearObjects_ = true; };

        /**
         * Reset the flag that enables point cloud filtration
         */
        void disableFilterNearObjects() { filterNearObjects_ = false; };

        void enableFilterOutliers() { filterOutliers_ = true; };

        void disableFilterOutliers() { filterOutliers_ = false; };

    private:

        bool filterNearObjects_ = false;
        bool filterOutliers_ = false;

        static void filterNearObjects(pcl::PointCloud<pcl::PointXYZ>::Ptr &data) {
            auto backup = data->points;
            data->points.clear();

            for (const auto &point: backup) {
                // Used lidars have minimal range of 2m, this formula is quicker than using powers
                if (std::abs(point.x) + std::abs(point.y) < 2.82) {
                    continue;
                }
                data->points.push_back(point);
            }
            data->width = data->size();
        }

        static void filterOutliers(pcl::PointCloud<pcl::PointXYZ>::Ptr &data) {

            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
            sor.setInputCloud(data);
            sor.setMeanK(50);
            sor.setStddevMulThresh(1.0);
            sor.filter(*data);
        }
    };

}
