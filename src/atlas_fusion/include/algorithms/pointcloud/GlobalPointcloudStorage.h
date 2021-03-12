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

#include "Context.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "algorithms/pointcloud/PointCloudProcessor.h"

namespace AtlasFusion::Algorithms {

    /**
     *  Class is designed as a container for asynchronous point cloud aggregation during the entire mapping session
     *  In this way the global point cloud map is created.
     */

    class GlobalPointcloudStorage {

    public:

        GlobalPointcloudStorage() = delete;

       /**
        * Constructor
        * @param context contains global services, like logger, TF tree or calibration loader
        * @param leafSize leaf size of the output point cloud
        */
        GlobalPointcloudStorage(Context& context, float leafSize)
                : context_{context}
                , processor_{context, leafSize} {

            globalStorage_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        }

        /**
         * Method accepts new point clouds and aggregates them into the global model
         * @param pc new points
         */
        void addMorePointsToGlobalStorage(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pc);

        /**
         * Getter for all the currently aggregated points.
         * @return donwsampled global point cloud map with a leaf size defined from constructor
         */
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> getEntirePointcloud();

    private:

        Context& context_;

        PointCloudProcessor processor_;
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> globalStorage_;
    };

}

