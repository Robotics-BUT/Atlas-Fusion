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

#include "algorithms/pointcloud/PointCloudAggregator.h"
#include "Timer.h"

namespace AutoDrive::Algorithms {

    void PointCloudAggregator::addPointCloudBatches(const std::vector<std::shared_ptr<DataModels::PointCloudBatch>> &batches) {
        for (const auto &batch: batches) {
            batchQueue_.push_back(batch);
            totalPoints_ += batch->getPointsSize();
        }
    }


    void PointCloudAggregator::filterOutBatches(uint64_t currentTime) {
        while (!batchQueue_.empty()) {
            auto timeDiff = static_cast<double>((currentTime - batchQueue_.front()->getTimestamp())) * 1e-9;
            if ((timeDiff > aggregationTime_)) {
                totalPoints_ -= batchQueue_.front()->getPointsSize();
                batchQueue_.pop_front();
            } else {
                break;
            }
        }
    }


    std::vector<std::shared_ptr<DataModels::PointCloudBatch>> PointCloudAggregator::getAllBatches() {
        std::vector<std::shared_ptr<DataModels::PointCloudBatch>> output;
        output.reserve(batchQueue_.size());

        for (auto it = batchQueue_.begin(); it < batchQueue_.end(); it++) {
            output.push_back(*it);
        }

        return output;
    }


    pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudAggregator::getAggregatedPointCloud() {
        Timer timer("getAggregatedPointCloud");

        pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
        if (batchQueue_.empty()) return output;

        uint32_t threads = context_.threadPool.get_thread_count();
        std::vector<std::future<pcl::PointCloud<pcl::PointXYZ>::Ptr>> outputFutures;
        outputFutures.resize(threads);
        uint32_t batchesPerThread = std::ceil(batchQueue_.size() / threads);

        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> inputClouds;
        inputClouds.resize(threads);

        for (uint32_t i = 0; i < threads; i++) {
            // Split point concatenation into multiple threads
            outputFutures[i] = context_.threadPool.submit([&, i]() {
                pcl::PointCloud<pcl::PointXYZ>::Ptr out(new pcl::PointCloud<pcl::PointXYZ>);

                uint32_t start = i * batchesPerThread;
                uint32_t end = (i + 1) * batchesPerThread;
                if (i == threads - 1) end += 1;

                for (uint32_t j = start; j < end; j++) {
                    if (j >= batchQueue_.size()) break;
                    pcl::concatenate(*out, *batchQueue_[j]->getTransformedPoints(), *out);
                }
                return out;
            });
        }
        context_.threadPool.wait_for_tasks();

        if (outputFutures.size() == 1) return outputFutures[0].get();

        for (auto &future: outputFutures) {
            pcl::concatenate(*output, *future.get(), *output);
        }

        return output;
    }


    pcl::PointCloud<pcl::PointXYZ>::Ptr
    PointCloudAggregator::getPointCloudCutout(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input, const rtl::BoundingBox3f &borders) {
        Timer timer("getPointCloudCutout");

        pcl::PointCloud<pcl::PointXYZ>::Ptr output{};
        output->reserve(input->size());

        for (const auto &point: *input) {
            if (point.x > borders.min().getElement(0) && point.x < borders.max().getElement(0) &&
                point.y > borders.min().getElement(1) && point.y < borders.max().getElement(1) &&
                point.z > borders.min().getElement(2) && point.z < borders.max().getElement(2)) {
                output->push_back(point);
            }
        }
        return output;
    }
}