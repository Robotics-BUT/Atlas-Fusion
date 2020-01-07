#pragma once

namespace AutoDrive::Visualizers {

    struct Color {

        explicit Color(float r, float g, float b)
                : red_{r}
                , green_{g}
                , blue_{b}
                , alpha_{1.0} { }

        explicit Color(float r, float g, float b, float a)
                : red_{r}
                , green_{g}
                , blue_{b} 
                , alpha_{a}{ }
        
        float red_;
        float green_;
        float blue_;
        float alpha_;
    };
}