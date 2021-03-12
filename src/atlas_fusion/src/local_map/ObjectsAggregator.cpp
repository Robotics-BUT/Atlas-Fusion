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

#include "local_map/ObjectsAggregator.h"
#include "munkres/munkres.h"

namespace AtlasFusion::LocalMap {

    std::vector<std::shared_ptr<const DataModels::LidarDetection>> ObjectsAggregator::aggregateLidarDetections(
            std::vector<std::shared_ptr<const DataModels::LidarDetection>> previousDetections,
            std::vector<std::shared_ptr<const DataModels::LidarDetection>> newDetections) const {

        std::vector<std::shared_ptr<const DataModels::LidarDetection>> output;

        // Edge cases
        if(newDetections.empty()) {
            output.reserve(previousDetections.size());
            for(const auto& detection: previousDetections) {
                if( detection->getTTL() > 1) {
                    output.emplace_back( std::make_shared<const DataModels::LidarDetection>(detection->getBoundingBox(), detection->getID(), detection->getTTL()-1));
                }
            }
            return output;
        }

        if(previousDetections.empty()) {
            return newDetections;
        }


        // Match detections
        auto matching = matchDetections(previousDetections, newDetections);

        // Fusing old and new together
        output = mergeDetections(previousDetections, newDetections, matching);
        return output;
    }


    std::vector<std::pair<unsigned, unsigned>> ObjectsAggregator::matchDetections(
            std::vector<std::shared_ptr<const DataModels::LidarDetection>> a,
            std::vector<std::shared_ptr<const DataModels::LidarDetection>> b) const {

        unsigned cols = static_cast<int>(a.size());
        unsigned rows = static_cast<int>(b.size());
        std::vector<float> costs(cols * rows, std::numeric_limits<float>::max());

        {int r = 0; for(const auto& newOne : b) {

                {int c = 0;for(const auto& previousOne : a) {
                        costs.at(r*cols + c) = static_cast<float>( -previousOne->getBoundingBox().intersectionOverUnion( newOne->getBoundingBox() ));
                        c++;
                    }}
                r++;
            }}

        auto f = [&](unsigned r, unsigned c) { return costs[r * cols + c]; };
        auto matching = Munkres::munkres_algorithm<float>(rows, cols, f);
        return matching;
    }

    std::vector<std::shared_ptr<const DataModels::LidarDetection>> ObjectsAggregator::mergeDetections(
            std::vector<std::shared_ptr<const DataModels::LidarDetection>> a,
            std::vector<std::shared_ptr<const DataModels::LidarDetection>> b,
            std::vector<std::pair<unsigned, unsigned>> matches) const {

        unsigned cols = static_cast<int>(a.size());
        unsigned rows = static_cast<int>(b.size());
        std::vector<std::shared_ptr<const DataModels::LidarDetection>> output;

        std::vector<bool> newMatched(rows, false);
        std::vector<bool> oldMatched(cols, false);
        for (const auto& match : matches) {
            output.emplace_back( std::make_shared<const DataModels::LidarDetection>(b.at(match.first)->getBoundingBox(), a.at(match.second)->getID()));
            newMatched.at(match.first) = true;
            oldMatched.at(match.second) = true;
        }

        for(size_t i = 0 ; i < newMatched.size() ; i++) {
            if ( !newMatched.at(i) ) {
                output.emplace_back( b.at(i) );
            }
        }

        for(size_t i = 0 ; i < oldMatched.size() ; i++) {
            if ( !oldMatched.at(i) ) {
                if( a.at(i)->getTTL() > 1) {
                    output.emplace_back( std::make_shared<const DataModels::LidarDetection>(
                            a.at(i)->getBoundingBox(),
                            a.at(i)->getID(),
                            a.at(i)->getTTL()-1));
                }
            }
        }
        return output;
    }
}