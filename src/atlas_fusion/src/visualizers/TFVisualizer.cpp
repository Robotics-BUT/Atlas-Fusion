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

#include "visualizers/TFVisualizer.h"

namespace AutoDrive::Visualizers {

    void TFVisualizer::updateOriginToRootTf(const rtl::RigidTf3D<double> &tf) {

        geometry_msgs::msg::TransformStamped tf_msg;

        tf_msg.header.stamp = node_->get_clock()->now();
        tf_msg.header.frame_id = frameTypeName(FrameType::kOrigin);
        tf_msg.child_frame_id = frameTypeName(context_.tfTree_.getRootFrameType());
        tf_msg.transform.translation.x = tf.trVecX();
        tf_msg.transform.translation.y = tf.trVecY();
        tf_msg.transform.translation.z = tf.trVecZ();
        tf_msg.transform.rotation.x = tf.rotQuaternion().x();
        tf_msg.transform.rotation.y = tf.rotQuaternion().y();
        tf_msg.transform.rotation.z = tf.rotQuaternion().z();
        tf_msg.transform.rotation.w = tf.rotQuaternion().w();
        rootToOriginBroadcaster_.sendTransform(tf_msg);
    }
}