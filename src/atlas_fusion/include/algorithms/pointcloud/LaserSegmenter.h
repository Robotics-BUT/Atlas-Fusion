#pragma once

#include "Context.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
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


#include "rtl/seg/IA_Segmenter.h"
#include "rtl/Vectorization.h"

namespace AtlasFusion::Algorithms {

    class LaserSegmenter {

    public:

        LaserSegmenter() = delete;

        /**
         * Constructor
         * @param context global services container, like loger, etc.
         * @param noOfLasers number of lasers per sensor
         * @param vectorizerSigma
         * @param segmenterStep
         * @param segmenterLowerBound
         * @param segmenterUpperBound
         * @param segmenterScaling
         */
        LaserSegmenter(Context& context, size_t noOfLasers, float vectorizerSigma, size_t segmenterStep,
                       float segmenterLowerBound, float segmenterUpperBound, float segmenterScaling)
        : context_{context}
        , noOfLasers_{noOfLasers}
        , vectorizerSigma_{vectorizerSigma}
        , segmenterStep_{segmenterStep}
        , segmenterLowerBound_{segmenterLowerBound}
        , segmenterUpperBound_{segmenterUpperBound}
        , segmenterScaling_{segmenterScaling} {

            vectorizer_.setSigma(vectorizerSigma_);

            segmenters_.emplace_back(segmenterStep_, segmenterLowerBound_, segmenterUpperBound_, segmenterScaling_);
            segmenters_.resize(noOfLasers_, segmenters_.front());

            lineSegments_.resize(noOfLasers_);
            lineSegmentsRoad_.resize(noOfLasers_);
        }

        /**
         * Method accepts the point cloud data from one laser, performs segmentation and stores the segmented
         * approximation lines into the internall storage
         * @param laserData
         */
        void onLaserData(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> laserData, size_t laserNo);

        /**
         * Getter of the approximations for single laser
         * @param laserNo laser number
         * @return shared pointer of the vector of the laser approximation
         */
        std::shared_ptr<std::vector<rtl::LineSegment3D<double>>> getApproximation(size_t laserNo);

        /**
         * Getter of the approximations for all lasers
         * @return shared pointer of the vector of the laser approximation
         */
        std::shared_ptr<std::vector<rtl::LineSegment3D<double>>> getAllApproximations();


        /**
         * Getter of the approximations of the road for single laser
         * @param laserNo laser number
         * @return shared pointer of the vector of the laser approximation
         */
        std::shared_ptr<std::vector<rtl::LineSegment3D<double>>> getRoad(size_t laserNo);

        /**
         * Getter of the approximations of the road for all lasers
         * @return shared pointer of the vector of the laser approximation
         */
        std::shared_ptr<std::vector<rtl::LineSegment3D<double>>> getAllRoads();

        /**
         * Clears point laser approximations from memory.
         */
        void clear();

    private:

        Context& context_;
        size_t noOfLasers_;

        float vectorizerSigma_;

        size_t segmenterStep_;
        float segmenterLowerBound_;
        float segmenterUpperBound_;
        float segmenterScaling_;

        std::vector<rtl::IA_Segmenter<rtl::Vector3D<double>>> segmenters_;
        rtl::VectorizerAFTLSProjections3D<double, double> vectorizer_;
        std::vector<std::vector<rtl::LineSegment3D<double>>> lineSegments_;
        std::vector<std::vector<rtl::LineSegment3D<double>>> lineSegmentsRoad_;
    };

}
