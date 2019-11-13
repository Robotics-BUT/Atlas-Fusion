#pragma once

#include <rtl/Vector3D.h>
#include <rtl/Quaternion.h>

namespace AutoDrive::DataLoader {

    struct TFFrame {
        rtl::Vector3D<double> translation_;
        rtl::Quaternion<double> rotation_;
        std::string name_;
    };
}