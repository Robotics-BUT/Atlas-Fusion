#pragma once

#include "Context.h"

namespace AutoDrive::Visualizers {

    /**
     * Visualization backend (ROS) implementations for visualizing telemetry text
     */
    class TelemetryVisualizer {

    public:

        TelemetryVisualizer() = delete;

        /**
         * Constructor
         * @param node ros node reference
         * @param context global services container (timestamps, logging, etc.)
         */
        TelemetryVisualizer(ros::NodeHandle& node, Context& context)
        : node_{node}
        , context_{context} {

        }

        void drawTelemetryAsText(std::string& telemetryText, std::string frame, std::string topic);

    private:

        ros::NodeHandle& node_;
        Context& context_;
        std::map<std::string, ros::Publisher> publishers_;

    };

}
