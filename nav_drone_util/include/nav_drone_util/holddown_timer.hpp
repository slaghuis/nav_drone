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

// A holddown timer to test if a given condition has been true for long enough

#pragma once

#include <chrono>   // for high_resolution_clock

namespace nav_drone_util {

class HolddownTimer {
  std::chrono::time_point<std::chrono::high_resolution_clock> start_time;
  int duration;
  bool running;
public:
  HolddownTimer(int dur): duration{dur}, running{false} { }
  
  bool test(bool condition) {
    if (!running && condition) {  // Start the timer
      running = true;
      start_time = std::chrono::high_resolution_clock::now();
    }
    
    if (running && !condition) { //condition failed again
        running = false;
    }
    
    auto current_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = current_time - start_time;
    
    return (running && (elapsed.count() >= (double) duration));
  }
  
};

} // namespace nav_drone_util
