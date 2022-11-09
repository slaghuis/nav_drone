// Copyright 2022 Eric Slaghuis
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// interprted from the work of Bradley J. Snyder <snyder.bradleyj@gmail.com>

#pragma once

#include <iostream>
#include <cmath>
#include <algorithm> //std::clamp

using namespace std;

class PID {
    float _dt, _max, _min, _Kp, _Kd, _Ki, _pre_error, _integral;
  public:
  
    // Kp - proportional gain
    // Ki - integral gain
    // Kd - derivative gain
    // dt - loop interval time
    // max - maximum value of manipulated variable
    // min - minimum value of manipulated variable 
    PID(float dt, float max, float min, float Kp, float Kd, float Ki)
      : _dt(dt), _max(max), _min(min), _Kp(Kp), _Kd(Kd), _Ki(Ki), _pre_error(0.0), _integral(0.0)
    { }

    float calculate(float setpoint, float pv)
    {
      // Calculate error
      float error = setpoint - pv;
  
      // Proportional term
      float Pout = _Kp * error;     
  
      // Integral Term
      _integral += error * _dt;
      float Iout = _Ki * _integral;  
  
      // Derivative Term
      float derivative = (error - _pre_error) / _dt;
      float Dout = _Kd * derivative;
  
      //Calculate total output
      float output = Pout + Iout + Dout;
  
      // Restrict to max/min
      output = std::clamp(output, _min, _max);
    
      _pre_error = error;
      
      return output;
    }
    
    void restart_control() 
    {
      _pre_error = 0.0;
      _integral = 0.0;
    }
};