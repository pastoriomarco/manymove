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

#ifndef MANYMOVE_CPP_TREES_HMI_HELPER_NODE_HPP
#define MANYMOVE_CPP_TREES_HMI_HELPER_NODE_HPP

#include <behaviortree_cpp_v3/blackboard.h>
#include <chrono>
#include <string>
#include <unordered_map>

namespace manymove_cpp_trees
{
inline void setHMIMessage(
  const BT::Blackboard::Ptr & blackboard,
  const std::string & key,
  const std::string & value,
  const std::string & color,
  double rate_hz = 5.0)
{
  using clock = std::chrono::steady_clock;
  struct State
  {
    clock::time_point last_time
    {
    };
    std::string last_value;
    std::string last_color;
  };

  static std::unordered_map<std::string, State> states;
  auto now = clock::now();
  auto period = std::chrono::duration<double>(1.0 / rate_hz);

  State & st = states[key];
  bool should_update = false;
  if (st.last_value != value || st.last_color != color) {
    should_update = true;
  }
  else if (now - st.last_time >= period) {
    should_update = true;
  }

  if (should_update) {
    blackboard->set(
      key + "message",
      value);
    blackboard->set(
      key + "message_color",
      color);
    st.last_time = now;
    st.last_value = value;
    st.last_color = color;
  }
}
} // namespace manymove_cpp_trees

#endif // MANYMOVE_CPP_TREES_HMI_HELPER_NODE_HPP
