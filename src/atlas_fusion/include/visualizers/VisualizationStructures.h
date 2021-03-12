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

#pragma once

namespace AtlasFusion::Visualizers {

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