#include "visualizers/TFVisualizer.h"
#include <sstream>

namespace AutoDrive::Visualizers {

    void TFVisualizer::addFrame(const std::string name, const rtl::Vector3D<double> trans, const rtl::Quaternion<double> rot) {

        if (frames_.find(name) != frames_.end()) {
            logger_.warning("Frame with name " + name + " already exists!");
            return;
        }

        DataLoader::TFFrame newFrame{trans, rot};
        frames_[name] = newFrame;
        keys_.push_back(name);
    }

}