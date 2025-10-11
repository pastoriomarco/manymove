// Copyright 2025 Flexin Group SRL
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Flexin Group SRL nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef MANYMOVE_CPP_TREES_BT_CONVERTERS_HPP
#define MANYMOVE_CPP_TREES_BT_CONVERTERS_HPP

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <geometry_msgs/msg/pose.hpp>
#include <sstream>
#include <stdexcept>

namespace BT
{
template<>
inline geometry_msgs::msg::Pose convertFromString(StringView str)
{
  geometry_msgs::msg::Pose pose;
  std::string s(str);

  // Simple parsing assuming the format:
  // "position: {x: 0.15, y: 0, z: 0}, orientation: {x: 0, y: 0, z: 0, w: 1}"
  try {
    // Parse position
    size_t pos_start = s.find("position:");
    if (pos_start == std::string::npos) {
      throw std::invalid_argument("Missing 'position' field.");
    }

    size_t x_pos = s.find("x:", pos_start);
    size_t y_pos = s.find("y:", pos_start);
    size_t z_pos = s.find("z:", pos_start);

    if (x_pos == std::string::npos || y_pos == std::string::npos || z_pos == std::string::npos) {
      throw std::invalid_argument("Incomplete 'position' fields.");
    }

    pose.position.x = std::stod(s.substr(x_pos + 2));
    pose.position.y = std::stod(s.substr(y_pos + 2));
    pose.position.z = std::stod(s.substr(z_pos + 2));

    // Parse orientation
    size_t ori_start = s.find("orientation:", pos_start);
    if (ori_start == std::string::npos) {
      throw std::invalid_argument("Missing 'orientation' field.");
    }

    size_t ox_pos = s.find("x:", ori_start);
    size_t oy_pos = s.find("y:", ori_start);
    size_t oz_pos = s.find("z:", ori_start);
    size_t ow_pos = s.find("w:", ori_start);

    if (ox_pos == std::string::npos || oy_pos == std::string::npos ||
        oz_pos == std::string::npos || ow_pos == std::string::npos) {
      throw std::invalid_argument("Incomplete 'orientation' fields.");
    }

    pose.orientation.x = std::stod(s.substr(ox_pos + 2));
    pose.orientation.y = std::stod(s.substr(oy_pos + 2));
    pose.orientation.z = std::stod(s.substr(oz_pos + 2));
    pose.orientation.w = std::stod(s.substr(ow_pos + 2));
  } catch (const std::exception& e) {
    throw BT::RuntimeError(std::string("Failed to parse Pose string: ") + e.what());
  }

  return pose;
}

template<>
inline std::vector<double> convertFromString(StringView str)
{
  std::vector<double> vec;
  std::string s(str);
  std::istringstream iss(s);
  char c;
  double value;

  // Expecting format: [x,y,z]
  if (!(iss >> c) || c != '[') {
    throw BT::RuntimeError("Failed to parse vector<double>: missing opening '['");
  }

  while (iss >> value) {
    vec.push_back(value);
    iss >> c;
    if (c == ']') {
      break;
    }
    if (c != ',') {
      throw BT::RuntimeError("Failed to parse vector<double>: expected ',' or ']'");
    }
  }

  return vec;
}

inline std::string convertToString(const std::vector<double>& vec)
{
  std::ostringstream oss;
  oss << "[";
  for (size_t i = 0; i < vec.size(); ++i) {
    oss << vec[i];
    if (i != vec.size() - 1) {
      oss << ",";
    }
  }
  oss << "]";
  return oss.str();
}

template<>
inline std::vector<std::string> convertFromString(StringView str)
{
  std::vector<std::string> vec;
  std::string s(str);
  std::istringstream iss(s);
  std::string value;

  // Expecting format: [str1,str2,str3]
  if (s.front() != '[' || s.back() != ']') {
    throw BT::RuntimeError("Failed to parse vector<string>: missing opening/closing brackets.");
  }

  // Remove the brackets
  s = s.substr(1, s.size() - 2);

  std::istringstream ss(s);
  while (std::getline(ss, value, ',')) {
    vec.push_back(value);
  }

  return vec;
}

inline std::string convertToString(const std::vector<std::string>& vec)
{
  std::ostringstream oss;
  oss << "[";
  for (size_t i = 0; i < vec.size(); ++i) {
    oss << vec[i];
    if (i < vec.size() - 1) {
      oss << ",";
    }
  }
  oss << "]";
  return oss.str();
}

template<>
inline std::string toStr<std::vector<std::string> >(std::vector<std::string> value)
{
  std::ostringstream oss;
  oss << "[";
  for (size_t i = 0; i < value.size(); ++i) {
    oss << value[i];
    if (i < value.size() - 1) {
      oss << ",";
    }
  }
  oss << "]";
  return oss.str();
}

}

#endif // MANYMOVE_CPP_TREES_BT_CONVERTERS_HPP
