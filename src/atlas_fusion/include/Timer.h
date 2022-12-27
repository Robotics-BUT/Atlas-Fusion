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

#include <chrono>
#include <string>
#include <utility>

struct Timer {

    using timePoint = std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>;

    timePoint _start, _end;
    std::chrono::duration<float> _duration{};

    std::string _name;
    float *_timerValue;

    Timer(std::string name, float *timerValue = nullptr) : _start(std::chrono::high_resolution_clock::now()), _name(std::move(name)), _timerValue(timerValue) {}

    ~Timer() {
        _end = std::chrono::high_resolution_clock::now();
        _duration = _end - _start;

        float ms = _duration.count() * 1000;
        if(_timerValue != nullptr) *_timerValue = ms;

        // Used to only show timers that take more than set amount
        if(ms > 1) {
            std::cout << "Execution of \"" << _name << "\" took: " << ms << " ms" << std::endl;
        }
    }
};