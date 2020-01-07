#include "visualizers/TFVisualizer.h"
#include <sstream>

namespace AutoDrive::Visualizers {

    void TFVisualizer::updateOriginToRootTf(rtl::Transformation3D<double> tf) {

        geometry_msgs::TransformStamped tf_msg;

        tf_msg.header.stamp = ros::Time::now();
        tf_msg.header.frame_id = LocalMap::Frames::kOrigin;
        tf_msg.child_frame_id = context_.tfTree_.getRootFrameName();
        tf_msg.transform.translation.x = tf.trX();
        tf_msg.transform.translation.y = tf.trY();
        tf_msg.transform.translation.z = tf.trZ();
        tf_msg.transform.rotation.x = tf.rotQuaternion().x();
        tf_msg.transform.rotation.y = tf.rotQuaternion().y();
        tf_msg.transform.rotation.z = tf.rotQuaternion().z();
        tf_msg.transform.rotation.w = tf.rotQuaternion().w();
        rootToOriginBroadcaster_.sendTransform(tf_msg);
    }
}