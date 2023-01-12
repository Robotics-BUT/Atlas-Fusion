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

#include "visualizers/TextStatusVisualizer.h"

namespace AutoDrive::Visualizers {


    void TextStatusVisualizer::drawStatusAsText(const std::string &statusText, const std::string &topic) {
        /*
        if (publishers_.count(topic) == 0) {
            publishers_[topic] = node_.advertise<jsk_rviz_plugins::OverlayText>(topic, 0);
        }

        std_msgs::ColorRGBA color;
        color.r = 1.0;
        color.g = 0.0;
        color.b = 0.0;
        color.a = 1.0;

        jsk_rviz_plugins::OverlayText textMsg;
        textMsg.width = 400;
        textMsg.height = 600;
        textMsg.left = 10;
        textMsg.top = 10;
        textMsg.text_size = 8;
        textMsg.line_width = 2;
        textMsg.font = "DejaVu Sans Mono";
        textMsg.fg_color = color;
        textMsg.text = statusText;

        publishers_[topic].publish(textMsg);
         */
    }
}
