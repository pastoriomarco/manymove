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

#include "manymove_hmi/ros2_worker.hpp"

#include <sstream>
#include <QMetaObject>
#include <algorithm>
#include <cctype>
#include <stdexcept>

using namespace std::chrono_literals;

Ros2Worker::Ros2Worker(
  const std::string&node_name, HmiGui*gui,
  const std::string&robot_prefix)
  : Node(node_name), gui_(gui), robot_prefix_(robot_prefix)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Ros2Worker node started with prefix: " << robot_prefix_);

  // Subscribe to the blackboard_status topic.
  subscription_ = this->create_subscription<std_msgs::msg::String>("blackboard_status", 10,
                                                                   std::bind(&Ros2Worker::
                                                                             statusCallback, this,
                                                                             std::placeholders::_1));

  // Create a client for the update_blackboard service.
  update_blackboard_client_ =
    this->create_client<manymove_msgs::srv::SetBlackboardValues>("update_blackboard");

  // Wait a little for the service.
  if (!update_blackboard_client_->wait_for_service(2s)) {
    RCLCPP_WARN(this->get_logger(),
                "Service 'update_blackboard' not available yet. Will still attempt calls.");
  }

  RCLCPP_INFO(this->get_logger(), "Ros2Worker fully initialized for prefix '%s'.",
              robot_prefix_.c_str());
}

void Ros2Worker::statusCallback(const std_msgs::msg::String::SharedPtr msg)
{
  const std::string data = msg->data;

  /* ---------- GUI buttons (unchanged) -------------------------- */
  const bool stop_execution =
    data.find("\"" + robot_prefix_ + "stop_execution\":true") != std::string::npos ||
    data.find("\"" + robot_prefix_ + "stop_execution\": true") != std::string::npos;
  const bool reset =
    data.find("\"" + robot_prefix_ + "reset\":true") != std::string::npos ||
    data.find("\"" + robot_prefix_ + "reset\": true") != std::string::npos;
  const bool collision_detected =
    data.find("\"" + robot_prefix_ + "collision_detected\":true") != std::string::npos ||
    data.find("\"" + robot_prefix_ + "collision_detected\": true") != std::string::npos;

  QMetaObject::invokeMethod(gui_, "updateStatus", Qt::QueuedConnection,
                            Q_ARG(QString, QString::fromStdString(robot_prefix_)),
                            Q_ARG(bool, stop_execution),
                            Q_ARG(bool, reset),
                            Q_ARG(bool, collision_detected));

  /* ---------- HMI keys ----------------------------------------- */
  AppModule*appModule = gui_->findChild<AppModule*>();
  if (!appModule) {
    return;
  }

  const auto&knownKeys = appModule->getKnownKeys();

  auto stripQuotes = [](std::string&s)
		     {
		       if (!s.empty() && s.front() == '"') {
			 s.erase(0, 1);
		       }
		       if (!s.empty() && s.back() == '"') {
			 s.pop_back();
		       }
		     };

  for (const auto&bk : knownKeys) {
    /* keys in the JSON are *not* prefixed – keep original pattern */
    const std::string pattern = "\"" + bk.key.toStdString() + "\":";
    size_t pos = data.find(pattern);
    if (pos == std::string::npos) {
      QMetaObject::invokeMethod(appModule, "updateField", Qt::QueuedConnection,
                                Q_ARG(QString, bk.key), Q_ARG(QString, QString()));
      continue;
    }

    /* move to the value start */
    size_t valStart = pos + pattern.length();
    while (valStart < data.size() && std::isspace(data[valStart])) {
      ++valStart;
    }

    std::string valueStr;

    /* ---------------- numeric DOUBLE ------------------------- */
    if (bk.type == "double") {
      size_t valEnd = data.find_first_of(",}", valStart);
      if (valEnd == std::string::npos) {
	valEnd = data.size();
      }
      valueStr = data.substr(valStart, valEnd - valStart);
      stripQuotes(valueStr);

      try {
	double d = std::stod(valueStr);
	valueStr = std::to_string(d);
      } catch (...) {
	valueStr.clear();
      }
    }
    /* ---------------- integer INT ---------------------------- */
    else if (bk.type == "int") {
      size_t valEnd = data.find_first_of(",}", valStart);
      if (valEnd == std::string::npos) {
	valEnd = data.size();
      }
      valueStr = data.substr(valStart, valEnd - valStart);
      stripQuotes(valueStr);

      try {
	int i = std::stoi(valueStr);
	valueStr = std::to_string(i);
      } catch (...) {
	valueStr.clear();
      }
    }
    /* ---------------- double array --------------------------- */
    else if (bk.type == "double_array") {
      if (data[valStart] == '"') {
	++valStart;
      }                   // skip leading quote
      if (data[valStart] == '[') {                   // keep the brackets
	size_t end = data.find(']', valStart);
	if (end != std::string::npos) {
	  valueStr = data.substr(valStart, end - valStart + 1);
	}
      }
      stripQuotes(valueStr);
    }
    /* ---------------- pose (JSON object) --------------------- */
    else if (bk.type == "pose") {
      if (data[valStart] == '"') {
	++valStart;                         // optional quote

      }
      if (data[valStart] == '{') {
	int braces = 0;
	size_t idx = valStart;
	do{
	  if (data[idx] == '{') {
	    ++braces;
	  }
	  if (data[idx] == '}') {
	    --braces;
	  }
	  ++idx;
	} while (idx < data.size() && braces);
	valueStr = data.substr(valStart, idx - valStart);
      }
      stripQuotes(valueStr);
    }
    /* ---------------- bool / string / default --------------- */
    else {
      size_t valEnd = data.find_first_of(",}", valStart);
      if (valEnd == std::string::npos) {
	valEnd = data.size();
      }
      valueStr = data.substr(valStart, valEnd - valStart);
      stripQuotes(valueStr);
    }

    /* push to GUI -------------------------------------------- */
    QMetaObject::invokeMethod(appModule, "updateField", Qt::QueuedConnection,
                              Q_ARG(QString, bk.key),
                              Q_ARG(QString, QString::fromStdString(valueStr)));
  }

  /* ---------- per-robot message -------------------------------- */
  auto findString = [&](const std::string&key) -> std::string
		    {
		      std::string pattern = "\"" + key + "\":";
		      size_t pos = data.find(pattern);
		      if (pos == std::string::npos) {
			return std::string();
		      }
		      pos += pattern.size();
		      while (pos < data.size() && std::isspace(data[pos])) {
			++pos;
		      }
		      if (pos >= data.size() || data[pos] != '"') {
			return std::string();
		      }
		      ++pos;
		      size_t end = data.find('"', pos);
		      if (end == std::string::npos) {
			return std::string();
		      }
		      return data.substr(pos, end - pos);
		    };

  const std::string msgKey = robot_prefix_ + "message";
  const std::string colorKey = robot_prefix_ + "message_color";
  std::string msgText = findString(msgKey);
  std::string msgColor = findString(colorKey);
  QMetaObject::invokeMethod(gui_, "updateRobotMessage", Qt::QueuedConnection,
                            Q_ARG(QString, QString::fromStdString(robot_prefix_)),
                            Q_ARG(QString, QString::fromStdString(msgText)),
                            Q_ARG(QString, QString::fromStdString(msgColor)));

  /* ---------- general message ---------------------------------- */
  std::string genMsg = findString("hmi_message");
  std::string genColor = findString("hmi_message_color");
  QMetaObject::invokeMethod(appModule, "updateGeneralMessage", Qt::QueuedConnection,
                            Q_ARG(QString, QString::fromStdString(genMsg)),
                            Q_ARG(QString, QString::fromStdString(genColor)));
}

void Ros2Worker::callStartExecution()
{
  auto request = std::make_shared<manymove_msgs::srv::SetBlackboardValues::Request>();

  request->key.push_back(robot_prefix_ + "stop_execution");
  request->value_type.push_back("bool");
  request->value_data.push_back("false");       // JSON "false"

  auto future = update_blackboard_client_->async_send_request(request);

  if (!update_blackboard_client_->wait_for_service(1s)) {
    RCLCPP_WARN(this->get_logger(), "callStartExecution() => service not available yet.");
  }
}

void Ros2Worker::callStopExecution()
{
  // "STOP" => set <prefix + "stop_execution"> = true
  auto request = std::make_shared<manymove_msgs::srv::SetBlackboardValues::Request>();
  request->key.push_back(robot_prefix_ + "stop_execution");
  request->value_type.push_back("bool");
  request->value_data.push_back("true");

  auto future = update_blackboard_client_->async_send_request(request);
  if (!update_blackboard_client_->wait_for_service(1s)) {
    RCLCPP_WARN(this->get_logger(), "callStopExecution() => service not available yet.");
  }
}

void Ros2Worker::callResetProgram()
{
  // Set both <prefix>stop_execution = true and <prefix>reset = true
  auto request = std::make_shared<manymove_msgs::srv::SetBlackboardValues::Request>();

  request->key.push_back(robot_prefix_ + "stop_execution");
  request->value_type.push_back("bool");
  request->value_data.push_back("true");

  request->key.push_back(robot_prefix_ + "reset");
  request->value_type.push_back("bool");
  request->value_data.push_back("true");

  auto future = update_blackboard_client_->async_send_request(request);
  if (!update_blackboard_client_->wait_for_service(1s)) {
    RCLCPP_WARN(this->get_logger(), "callResetProgram() => service not available yet.");
  }
}
