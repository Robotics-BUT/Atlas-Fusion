#pragma once

namespace AutoDrive::Visualizers {

    /**
     * Visualization color structure
     */
    struct Color {

        /**
         * Constructor
         * @param r red
         * @param g green
         * @param b blue
         */
        explicit Color(float r, float g, float b)
                : red_{r}
                , green_{g}
                , blue_{b}
                , alpha_{1.0} { }

        /**
         * Constructor
         * @param r red
         * @param g green
         * @param b blue
         * @param a alpha
         */
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