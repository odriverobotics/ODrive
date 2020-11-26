/* MIT License

Copyright (c) 2016-2021 ODrive Robotics, Inc

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE. */

#ifndef _TRAP_TRAJ_H
#define _TRAP_TRAJ_H

class TrapezoidalTrajectory {
public:
    struct Config_t {
        float vel_limit = 2.0f;   // [turn/s]
        float accel_limit = 0.5f; // [turn/s^2]
        float decel_limit = 0.5f; // [turn/s^2]
    };
    
    struct Step_t {
        float Y;
        float Yd;
        float Ydd;
    };

    bool planTrapezoidal(float Xf, float Xi, float Vi,
                         float Vmax, float Amax, float Dmax);
    Step_t eval(float t);

    Axis* axis_ = nullptr;  // set by Axis constructor
    Config_t config_;

    float Xi_;
    float Xf_;
    float Vi_;

    float Ar_;
    float Vr_;
    float Dr_;

    float Ta_;
    float Tv_;
    float Td_;
    float Tf_;

    float yAccel_;

    float t_;
};

#endif