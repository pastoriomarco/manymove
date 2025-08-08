#ifndef MANYMOVE_CPP_TREES_HMI_HELPER_NODE_HPP
#define MANYMOVE_CPP_TREES_HMI_HELPER_NODE_HPP

#include <behaviortree_cpp_v3/blackboard.h>
#include <chrono>
#include <string>
#include <unordered_map>

namespace manymove_cpp_trees
{
inline void setHMIMessage(const BT::Blackboard::Ptr &blackboard,
                          const std::string &key,
                          const std::string &value,
                          const std::string &color,
                          double rate_hz = 5.0)
{
    using clock = std::chrono::steady_clock;
    struct State
    {
        clock::time_point last_time{};
        std::string last_value;
        std::string last_color;
    };

    static std::unordered_map<std::string, State> states;
    auto now = clock::now();
    auto period = std::chrono::duration<double>(1.0 / rate_hz);

    State &st = states[key];
    bool should_update = false;
    if (st.last_value != value || st.last_color != color)
    {
        should_update = true;
    }
    else if (now - st.last_time >= period)
    {
        should_update = true;
    }

    if (should_update)
    {
        blackboard->set(key + "message", value);
        blackboard->set(key + "message_color", color);
        st.last_time = now;
        st.last_value = value;
        st.last_color = color;
    }
}
} // namespace manymove_cpp_trees

#endif // MANYMOVE_CPP_TREES_HMI_HELPER_NODE_HPP