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

#include "Context.h"

namespace AtlasFusion {

    Context Context::getEmptyContext() {
        LogService logger("", LogService::LogLevel::Off, false, false);
        LocalMap::TFTree tfTree("", logger);
        std::string emptyPath = "";
        return Context(logger, tfTree, emptyPath, FunctionalityFlags{});
    }

    Context::timePoint Context::getHighPrecisionTime() {
        auto time = std::chrono::high_resolution_clock::now();
        return time;
    }

    double Context::highPrecisionTimeToMilliseconds( std::chrono::duration<long, std::ratio<1, 1000000000>> t) {
        return static_cast<double>(t.count()) * 1e-6;
    }

}