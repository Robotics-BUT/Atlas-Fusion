#pragma once

#include <opencv2/opencv.hpp>
#include "Context.h"
#include "algorithms/pointcloud/SphericalPoint.h"

namespace AtlasFusion::Algorithms {

    class SimpleImageHandler {



    public:

        SimpleImageHandler(Context& context)
                : context_{context}
                , maxDist_{50.0f} {

        }

        cv::Mat convertLeftFrontRGBToIrFieldOfView(cv::Mat) const;

        /**
         * Method renders depth map by drawing given points into the image plane.
         * @param points2D image coordinates (u, v) of the rendered points
         * @param points3D 3D space coordinates (x, y, z) of the rendered points (z is used)
         * @param imgWidth rendered image widht
         * @param imgHeight rendered image height
         * @return rendered depth image
         */
        cv::Mat renderLidarPointsToImg(std::vector<cv::Point2f>, std::vector<cv::Point3f>, int imgWidth, int imgHeightm, size_t pointSize=1, bool handle_height = false);

        cv::Mat renderSphericalPointsToImg(std::vector<SphericalPoint<float>>, float thetaMargin, float phiMargin, float range, float angular_step, size_t pointSize=1);

    private:

        Context& context_;
        float maxDist_;

        uint8_t distanceToColor(float dist);
        float pointLenght(cv::Point3f&);
        float deg2rad(float deg);
        float rad2deg(float rad);
    };
}
