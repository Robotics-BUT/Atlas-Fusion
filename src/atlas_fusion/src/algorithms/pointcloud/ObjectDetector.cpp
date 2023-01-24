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

namespace AutoDrive::Algorithms {


    std::vector<std::shared_ptr<DataModels::LidarDetection>> ObjectDetector::detectObstacles(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pc) {
        Timer t("Lidar obstacle detection");

        std::vector<std::shared_ptr<DataModels::LidarDetection>> output{};
        std::vector<std::future<DataModels::LidarDetection>> outputFutures{};
        if (pc->empty()) {
            return output;
        }

        std::vector<pcl::PointIndices> cluster_indices;
        {
            Timer t("EuclideanClusterExtraction");

            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

            pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
            ec.setClusterTolerance(0.50); // 20cm
            ec.setMinClusterSize(20);
            ec.setMaxClusterSize(100000);
            ec.setSearchMethod(tree);
            ec.setInputCloud(pc);
            ec.extract(cluster_indices);
        }
        if (cluster_indices.empty()) return output;

        outputFutures.resize(cluster_indices.size());
        output.reserve(cluster_indices.size());

        {
            Timer t("Object detection");

            static size_t detectionID = 0;
            for (size_t i = 0; i < cluster_indices.size(); i++) {
                outputFutures[i] = context_.threadPool_.submit([&, i]() {
                    auto it = cluster_indices[i];
                    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
                    cloud_cluster->points.reserve(cluster_indices.size());
                    cloud_cluster->width = cluster_indices.size();
                    cloud_cluster->height = 1;
                    for (const auto &pit: it.indices) {
                        cloud_cluster->points.emplace_back(pc->points[pit]);
                    }

                    pcl::MomentOfInertiaEstimation<pcl::PointXYZ> featureExtractor;
                    featureExtractor.setInputCloud(cloud_cluster);
                    featureExtractor.compute();

                    std::vector<float> eccentricity;
                    pcl::PointXYZ minPointOBB, maxPointOBB, positionOBB;
                    Eigen::Matrix3f rotationalMatrixOBB;

                    featureExtractor.getOBB(minPointOBB, maxPointOBB, positionOBB, rotationalMatrixOBB);

                    auto euler = rotationalMatrixOBB.eulerAngles(0, 1, 2);
                    Eigen::Quaternionf q(Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ()));
                    rtl::Quaterniond quat(Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z()));

                    return DataModels::LidarDetection(
                            rtl::BoundingBox3d{
                                    {positionOBB.x + minPointOBB.x, positionOBB.y + minPointOBB.y, positionOBB.z + minPointOBB.z},
                                    {positionOBB.x + maxPointOBB.x, positionOBB.y + maxPointOBB.y, positionOBB.z + maxPointOBB.z}
                            },
                            quat,
                            detectionID++
                    );
                });
            }

            for (auto &outputFuture: outputFutures) {
                outputFuture.wait();

                auto det = outputFuture.get();
                auto detBB = det.getBoundingBox();
                double xLength = abs(detBB.max().x() - detBB.min().x());
                double yLength = abs(detBB.max().y() - detBB.min().y());
                double zLength = abs(detBB.max().z() - detBB.min().z());

                // Filter out big obstacles
                if(xLength > 10.0 || yLength > 10.0 ||  zLength < 0.5 ) {
                    continue;
                }

                output.emplace_back(std::make_shared<DataModels::LidarDetection>(det));
            }
        }
        return output;
    }
}
