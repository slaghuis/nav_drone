#pragma once
#include <vector>

// Custom type
struct Pose3D
{
    double x, y, z, theta;
};


inline void SleepMS(int ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

// It is recommended (or, in some cases, mandatory) to define a template
// specialization of convertFromString that converts a string to Position2D.
namespace BT
{
template <> inline Pose3D convertFromString(StringView str)
{
    // real numbers separated by semicolons
    auto parts = splitString(str, ';');
    if (parts.size() != 4)
    {
        throw RuntimeError("invalid input)");
    }
    else{
        Pose3D output;
        output.x     = convertFromString<double>(parts[0]);
        output.y     = convertFromString<double>(parts[1]);
        output.z     = convertFromString<double>(parts[2]);
        output.theta = convertFromString<double>(parts[3]);
        return output;
    }
}

template <> inline std::vector<Pose3D> convertFromString(StringView str)
{
    // x;y;z;theta|x;y;z;theta
    std::vector<Pose3D> mult;
    auto points = splitString(str, '|');
    mult.reserve(points.size());
    
    for (std::size_t i=0; i<points.size(); i++)
    {
      auto parts = splitString(points[i], ';');
      if (parts.size() != 4)
      {
        throw RuntimeError("invalid input)");
      }
      else{
          Pose3D output;
          output.x     = convertFromString<double>(parts[0]);
          output.y     = convertFromString<double>(parts[1]);
          output.z     = convertFromString<double>(parts[2]);
          output.theta = convertFromString<double>(parts[3]);
          
          mult.push_back(output);
      }
    }
    
    return mult;
}


} // end namespace BT
