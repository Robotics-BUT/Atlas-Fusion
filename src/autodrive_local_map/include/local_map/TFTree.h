#pragma once
#include "LogService.h"
#include <rtl/Transformation3D.h>
#include <map>


namespace AutoDrive::LocalMap {

    /**
     * Transformation Tree holds and allows simple searching in the transformation graph between the sensor frames, and
     * the world origin. Currently TF Tree supports only the root frame and 1 level of child-nodes and thransfomrations
     * between them.
     */
    class TFTree {

    public:

        /**
         * Constructor
         * @param rootFrameName the central frame name
         * @param logger logger service instance
         */
        TFTree(std::string rootFrameName, LogService& logger)
        : rootFrameName_(std::move(rootFrameName))
        , logger_(logger) {

        }

        /**
         * Methods allows to add new child-frame under the root frame level
         * @param tf new child transformations
         * @param name new frame name
         */
        void addFrame(rtl::Transformation3Dd tf, std::string name);

        /**
         * Method returns transformation between the root frame the the child frame
         * @param frameName child frame name
         * @return child transformation
         */
        rtl::Transformation3Dd getTransformationForFrame(const std::string& frameName);

        /**
         * Method returns the vector of all child frame names.
         * @return all child frame names
         */
        const std::vector<std::string> getFrameNames() {return frameNames_;};

        /**
         * Getter for root frame name
         * @return root frame name
         */
        const std::string& getRootFrameName() {return rootFrameName_;};

        /**
         * Method estimates fransformation between two child frames.
         * @param srcPoint 3D point in the source coordinate systems (frame)
         * @param source source frame name
         * @param destination destination frame name
         * @return returns the point transformed from the original coordinate system to the new one.
         */
        rtl::Vector3D<double> transformPointFromFrameToFrame(rtl::Vector3D<double>, const std::string& source, const std::string& destination);

    protected:

        std::string rootFrameName_;
        std::vector<std::string> frameNames_{};
        std::unordered_map<std::string, rtl::Transformation3Dd> frameMap_{};
        LogService& logger_;

        const std::unordered_map<std::string, rtl::Transformation3Dd>& getTree() {return frameMap_;};
    };

}