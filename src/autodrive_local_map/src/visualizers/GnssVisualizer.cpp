#include "visualizers/GnssVisualizer.h"

#include "local_map/Frames.h"

namespace AutoDrive::Visualizers {


    void GnssVisualizer::drawGnssPose(const std::shared_ptr<DataModels::GnssPoseDataModel> data) const {

        visualization_msgs::Marker textMsg;
        textMsg.header.frame_id = LocalMap::Frames::kGnssAntennaRear;
        textMsg.header.stamp = ros::Time::now();
        textMsg.pose.position.x = 0;
        textMsg.pose.position.y = 0;
        textMsg.pose.position.z = -1.0;
        textMsg.scale.x = 0.2;
        textMsg.scale.y = textMsg.scale.x;
        textMsg.scale.z = textMsg.scale.x;
        textMsg.color.a = 1.0;
        textMsg.color.r = 0.0;
        textMsg.color.g = 1.0;
        textMsg.color.b = 0.0;
        textMsg.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        textMsg.action = visualization_msgs::Marker::ADD;

        std::stringstream ss;
        ss << std::setprecision(10)  << "Lat: " << data->getLatitude() << "\n"
           << "Lon: " << data->getLongitude() << std::endl
           << std::setprecision(4) << "Alt: " << data->getAltitude() << "\n"
           << "Azim: " << data->getAzimut();
        textMsg.text = ss.str();

        gnssPublisher_.publish(textMsg);
    }

}