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

#include "algorithms/pointcloud/ObjectDetector.h"
#include "Timer.h"

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/impl/search.hpp>

namespace AutoDrive::Algorithms {


    std::vector<std::shared_ptr<DataModels::LidarDetection>> ObjectDetector::detectObstacles(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pc) {
        Timer t("detectObstacles");

        std::vector<std::shared_ptr<DataModels::LidarDetection>> output{};
        std::vector<std::future<DataModels::LidarDetection>> outputFutures{};
        if (pc->empty()) {
            return output;
        }

        // Creating the KdTree object for the search method of the extraction
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(pc);


        //TODO: Waaay to slow -> EuclideanClusterExtraction can take up to 100 ms
        std::vector<pcl::PointIndices> cluster_indices;
        {
            Timer t("EuclideanClusterExtraction");
            pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
            ec.setClusterTolerance(0.50); // 20cm
            ec.setMinClusterSize(20);
            ec.setMaxClusterSize(100000);
            ec.setSearchMethod(tree);
            ec.setInputCloud(pc);
            ec.extract(cluster_indices);
        }
        if(cluster_indices.empty()) return output;

        outputFutures.resize(cluster_indices.size());
        output.reserve(cluster_indices.size());

        static size_t detectionID = 0;
        for (size_t i = 0; i < cluster_indices.size(); i++) {
            outputFutures[i] = context_.threadPool_.submit([&cluster_indices, &pc, i]() {
                auto it = cluster_indices[i];
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);

                rtl::Vector3D<double> minPoint{NAN, NAN, NAN};
                rtl::Vector3D<double> maxPoint{NAN, NAN, NAN};

                //TODO: Don't know why, but sometimes point at index [0] is {0,0,0}
                for (const auto &pit: it.indices) {
                    const auto &point = pc->points[pit];

                    if (pit != it.indices[3]) {
                        if (minPoint.x() > point.x) { minPoint.setX(point.x); }
                        if (minPoint.y() > point.y) { minPoint.setY(point.y); }
                        if (minPoint.z() > point.z) { minPoint.setZ(point.z); }
                        if (maxPoint.x() < point.x) { maxPoint.setX(point.x); }
                        if (maxPoint.y() < point.y) { maxPoint.setY(point.y); }
                        if (maxPoint.z() < point.z) { maxPoint.setZ(point.z); }
                    } else {
                        minPoint.setX(point.x);
                        minPoint.setY(point.y);
                        minPoint.setZ(point.z);
                        maxPoint.setX(point.x);
                        maxPoint.setY(point.y);
                        maxPoint.setZ(point.z);
                    }
                }

                return DataModels::LidarDetection(rtl::BoundingBox3d{minPoint, maxPoint}, detectionID++);
            });
        }

        for (auto &outputFuture: outputFutures) {
            outputFuture.wait();
            output.emplace_back(std::make_shared<DataModels::LidarDetection>(outputFuture.get()));
        }

        return output;
    }
}
